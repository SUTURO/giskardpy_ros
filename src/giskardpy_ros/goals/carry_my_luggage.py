from __future__ import division

from time import sleep
from typing import List, Optional, Dict

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker

from giskardpy import casadi_wrapper as cas
from giskardpy.data_types.data_types import Derivatives, ColorRGBA
from giskardpy.data_types.exceptions import GoalInitalizationException, ExecutionException
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.model.joints import OmniDrive
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.motion_statechart.monitors.monitors import Monitor, EndMotion
from giskardpy.motion_statechart.tasks.task import WEIGHT_BELOW_CA, Task
from giskardpy.symbol_manager import symbol_manager
from giskardpy.utils.decorators import clear_memo
from giskardpy_ros.tasks.vfh_task import VFHMoveDir
from giskardpy_ros.tree.blackboard_utils import raise_to_blackboard
from giskardpy_ros.utils.VFH import VectorFieldHistogram


class CarryMyBullshit(Goal):
    trajectory: np.ndarray = np.array([0, 0])
    traj_data: List[np.ndarray] = None
    thresholds: np.ndarray = None
    thresholds_pc: np.ndarray = None
    human_point: PointStamped = None
    pub: rospy.Publisher = None
    laser_sub: rospy.Subscriber = None
    point_cloud_laser_sub: rospy.Subscriber = None
    target_sub: rospy.Subscriber = None
    traj_flipped: bool = False
    last_scan: LaserScan = None
    last_scan_pc: LaserScan = None

    def __init__(self,
                 name: str,
                 patrick_topic_name: str = '/human_pose',
                 laser_topic_name: str = '/hsrb/base_scan',
                 point_cloud_laser_topic_name: Optional[str] = None,
                 odom_joint_name: str = 'brumbrum',
                 root_link: Optional[str] = None,
                 camera_link: str = 'head_rgbd_sensor_link',
                 distance_to_target_stop_threshold: float = 0.70,
                 cone_theta: float = 0.1,
                 laser_scan_age_threshold: float = 2,
                 laser_distance_threshold: float = 0.5,
                 laser_distance_threshold_width: float = 0.8,
                 laser_avoidance_angle_cutout: float = np.pi / 4,
                 laser_avoidance_sideways_buffer: float = 0.04,
                 base_orientation_threshold: float = np.pi / 16,
                 wait_for_patrick_timeout: int = 30,
                 max_rotation_velocity: float = 0.5,
                 max_rotation_velocity_head: float = 1.4,  # how to make that faster??
                 max_translation_velocity: float = 0.38,
                 traj_tracking_radius: float = 0.5,
                 height_for_camera_target: float = 1,
                 laser_frame_id: str = 'base_range_sensor_link',
                 target_age_threshold: float = 2,
                 target_age_exception_threshold: float = 5,
                 clear_path: bool = False,
                 drive_back: bool = False,
                 enable_laser_avoidance: bool = True):
        super().__init__(name=name)
        self.current_target = np.array([0, 0, 0, 1])
        self.closest_point = np.array([0, 0, 0, 1])
        self.vfh = VectorFieldHistogram(max_range=5.0,
                                        grid_size=0.1,
                                        sector_angle=5,
                                        obstacle_threshold=8,
                                        s_max=12,
                                        input_topic="/hsrb/base_scan")
        root_V_goal_angle = None
        if drive_back:
            get_middleware().loginfo('driving back')
        self.end_of_traj_reached = False
        self.enable_laser_avoidance = enable_laser_avoidance
        if CarryMyBullshit.pub is None:
            CarryMyBullshit.pub = rospy.Publisher('~visualization_marker_array', MarkerArray)
        self.laser_topic_name = laser_topic_name
        if point_cloud_laser_topic_name == '':
            self.point_cloud_laser_topic_name = None
        else:
            self.point_cloud_laser_topic_name = point_cloud_laser_topic_name
        self.laser_distance_threshold_width = laser_distance_threshold_width / 2
        self.last_target_age = 0
        self.closest_laser_left = self.laser_distance_threshold_width
        self.closest_laser_right = -self.laser_distance_threshold_width
        self.closest_laser_reading = 0
        self.closest_laser_left_pc = self.laser_distance_threshold_width
        self.closest_laser_right_pc = -self.laser_distance_threshold_width
        self.closest_laser_reading_pc = 0
        self.laser_frame = god_map.world.search_for_link_name(laser_frame_id)
        self.last_scan = LaserScan()
        self.last_scan.header.stamp = rospy.get_rostime()
        self.last_scan_pc = LaserScan()
        self.last_scan_pc.header.stamp = rospy.get_rostime()
        self.laser_scan_age_threshold = laser_scan_age_threshold
        self.laser_avoidance_angle_cutout = laser_avoidance_angle_cutout
        self.laser_avoidance_sideways_buffer = laser_avoidance_sideways_buffer
        self.base_orientation_threshold = base_orientation_threshold
        self.odom_joint_name = god_map.world.search_for_joint_name(odom_joint_name)
        self.odom_joint: OmniDrive = god_map.world.get_joint(self.odom_joint_name)
        self.target_age_threshold = target_age_threshold
        self.target_age_exception_threshold = target_age_exception_threshold
        if root_link is None:
            self.root = god_map.world.root_link_name
        else:
            self.root = god_map.world.search_for_link_name(root_link)
        self.camera_link = god_map.world.search_for_link_name(camera_link)
        self.tip_V_camera_axis = cas.Vector3()
        self.tip_V_camera_axis.z = 1
        self.tip = self.odom_joint.child_link_name
        self.odom = self.odom_joint.parent_link_name
        self.tip_V_pointing_axis = cas.Vector3()
        self.tip_V_pointing_axis.x = 1
        self.max_rotation_velocity = max_rotation_velocity
        self.max_rotation_velocity_head = max_rotation_velocity_head
        self.max_translation_velocity = max_translation_velocity
        self.weight = WEIGHT_BELOW_CA
        self.min_distance_to_target = distance_to_target_stop_threshold
        self.laser_distance_threshold = laser_distance_threshold
        self.traj_tracking_radius = traj_tracking_radius
        self.interpolation_step_size = 0.05
        self.max_temp_distance = int(self.traj_tracking_radius / self.interpolation_step_size)
        self.human_point = PointStamped()
        self.height_for_camera_target = height_for_camera_target
        self.drive_back = drive_back
        if clear_path or (not self.drive_back and CarryMyBullshit.trajectory is None):
            CarryMyBullshit.trajectory = np.array(self.get_current_point(), ndmin=2)
        if clear_path or CarryMyBullshit.traj_data is None:
            CarryMyBullshit.traj_data = [self.get_current_point()]
        if clear_path:
            CarryMyBullshit.traj_flipped = False
            get_middleware().loginfo('cleared old path')
        if CarryMyBullshit.laser_sub is None:
            CarryMyBullshit.laser_sub = rospy.Subscriber(self.laser_topic_name, LaserScan, self.laser_cb, queue_size=10)
        if CarryMyBullshit.point_cloud_laser_sub is None and self.point_cloud_laser_topic_name is not None:
            CarryMyBullshit.point_cloud_laser_sub = rospy.Subscriber(self.point_cloud_laser_topic_name,
                                                                     LaserScan, self.point_cloud_laser_cb,
                                                                     queue_size=10)
        self.publish_tracking_radius()
        self.publish_distance_to_target()
        if not self.drive_back:
            if CarryMyBullshit.target_sub is None:
                CarryMyBullshit.target_sub = rospy.Subscriber(patrick_topic_name, PointStamped, self.target_cb,
                                                              queue_size=10)
            rospy.sleep(0.5)
            for i in range(int(wait_for_patrick_timeout)):
                if CarryMyBullshit.trajectory.shape[0] > 5:
                    break
                print(f'waiting for at least 5 traj points, current length {len(CarryMyBullshit.trajectory)}')
                rospy.sleep(1)
            else:
                raise GoalInitalizationException(
                    f'didn\'t receive enough points after {wait_for_patrick_timeout}s')
            get_middleware().loginfo(f'waiting for one more target point for {wait_for_patrick_timeout}s')
            rospy.wait_for_message(patrick_topic_name, PointStamped, rospy.Duration(wait_for_patrick_timeout))
            get_middleware().loginfo('received target point.')

        else:
            if not CarryMyBullshit.traj_flipped:
                CarryMyBullshit.trajectory = np.flip(CarryMyBullshit.trajectory, axis=0)
                CarryMyBullshit.traj_flipped = True
            self.publish_trajectory()

        # %% real shit
        root_T_bf = god_map.world.compose_fk_expression(self.root, self.tip)
        root_T_odom = god_map.world.compose_fk_expression(self.root, self.odom)
        root_T_camera = god_map.world.compose_fk_expression(self.root, self.camera_link)
        root_P_bf = root_T_bf.to_position()

        min_left_violation1 = symbol_manager.get_symbol(self.ref_str + '.closest_laser_left')
        min_right_violation1 = symbol_manager.get_symbol(self.ref_str + '.closest_laser_right')
        closest_laser_reading1 = symbol_manager.get_symbol(self.ref_str + '.closest_laser_reading')
        min_left_violation2 = symbol_manager.get_symbol(self.ref_str + '.closest_laser_left_pc')
        min_right_violation2 = symbol_manager.get_symbol(self.ref_str + '.closest_laser_right_pc')
        closest_laser_reading2 = symbol_manager.get_symbol(self.ref_str + '.closest_laser_reading_pc')

        closest_laser_left = cas.min(min_left_violation1, min_left_violation2)
        closest_laser_right = cas.max(min_right_violation1, min_right_violation2)
        closest_laser_reading = cas.min(closest_laser_reading1, closest_laser_reading2)

        if not self.drive_back:
            last_target_age = symbol_manager.get_symbol(self.ref_str + '.last_target_age')
            target_lost = Monitor(name='target out of sight')
            self.add_monitor(target_lost)
            target_lost.observation_expression = cas.greater_equal(last_target_age, self.target_age_threshold)

        # todo reference new self attribute instead of function
        next_x = self.current_target[0]
        next_y = self.current_target[1]
        closest_x = self.closest_point[0]
        closest_y = self.closest_point[1]
        # next_x = symbol_manager.get_symbol(self.ref_str + '.get_current_target()[\'next_x\']') --tbr
        # next_y = symbol_manager.get_symbol(self.ref_str + '.get_current_target()[\'next_y\']') --tbr
        # closest_x = symbol_manager.get_symbol(self.ref_str + '.get_current_target()[\'closest_x\']')
        # closest_y = symbol_manager.get_symbol(self.ref_str + '.get_current_target()[\'closest_y\']')
        # tangent_x = god_map.to_expr(self._get_identifier() + ['get_current_target', tuple(), 'tangent_x'])
        # tangent_y = god_map.to_expr(self._get_identifier() + ['get_current_target', tuple(), 'tangent_y'])
        clear_memo(self.get_current_target)
        laser_P_goal_point = symbol_manager.get_expr(self.ref_str +
                                                     '.current_target',
                                                     input_type_hint=np.ndarray,
                                                     output_type_hint=cas.Point3)
        root_P_goal_point = god_map.world.compose_fk_expression(self.root, self.laser_frame).dot(laser_P_goal_point)
        root_P_closest_point = symbol_manager.get_expr(self.ref_str +
                                                       '.closest_point',
                                                       input_type_hint=np.ndarray,
                                                       output_type_hint=cas.Point3)
        while self.vfh.direction_vector is None:
            sleep(0.5)
        root_V_goal_angle = symbol_manager.get_expr(self.ref_str + '.vfh.direction_vector',
                                                    input_type_hint=np.ndarray, output_type_hint=cas.Vector3)
        # tangent = root_P_goal_point - root_P_closest_point
        # root_V_tangent = cas.Vector3([tangent.x, tangent.y, 0])
        tip_V_pointing_axis = self.tip_V_pointing_axis
        # print(root_V_goal_angle)
        if self.drive_back:
            map_P_human = root_P_goal_point
        else:
            map_P_human = cas.Point3((symbol_manager.get_symbol(self.ref_str + '.human_point.point.x'),
                                      symbol_manager.get_symbol(self.ref_str + '.human_point.point.y'),
                                      symbol_manager.get_symbol(self.ref_str + '.human_point.point.z')))
            map_P_human_projected = map_P_human
            map_P_human_projected.z = 0

        # %% orient to goal
        orient_to_goal = Task(name='orient to goal')
        self.add_task(orient_to_goal)
        _, _, map_odom_angle = root_T_odom.to_rotation().to_rpy()
        odom_current_angle = self.odom_joint.yaw.get_symbol(Derivatives.position)
        map_current_angle = map_odom_angle + odom_current_angle
        if self.drive_back:
            root_V_tip_to_closest = root_P_bf - root_P_closest_point
            root_P_between_tip_and_closest = root_P_closest_point + root_V_tip_to_closest / 2
            root_V_goal_axis = root_P_goal_point - root_P_between_tip_and_closest
            root_V_goal_axis2 = cas.Vector3(root_V_goal_axis)
            root_V_goal_axis2.scale(1)
            map_P_human = map_P_human + root_V_goal_axis2 * 1.5
        else:
            root_V_goal_axis = map_P_human_projected - root_P_bf
        distance_to_human = cas.norm(root_V_goal_axis)
        root_V_goal_axis.scale(1)
        root_V_pointing_axis = root_T_bf.dot(tip_V_pointing_axis)
        root_V_pointing_axis.vis_frame = self.tip
        root_V_goal_axis.vis_frame = self.tip
        map_goal_angle = cas.angle_between_vector(cas.Vector3([1, 0, 0]), root_V_goal_axis)
        map_goal_angle = cas.if_greater(root_V_goal_axis.y, 0, map_goal_angle, -map_goal_angle)
        map_angle_error = cas.shortest_angular_distance(map_current_angle, map_goal_angle)
        if self.drive_back:
            buffer = 0
        else:
            buffer = self.base_orientation_threshold
        ll = map_angle_error - buffer
        ul = map_angle_error + buffer
        orient_to_goal.add_inequality_constraint(reference_velocity=self.max_rotation_velocity,
                                                 lower_error=ll,
                                                 upper_error=ul,
                                                 weight=self.weight,
                                                 task_expression=map_current_angle,
                                                 name='rot')

        # %% look at goal
        camera_V_camera_axis = cas.Vector3(self.tip_V_camera_axis)
        root_V_camera_axis = root_T_camera.dot(camera_V_camera_axis)
        root_P_camera = root_T_camera.to_position()
        map_P_human.z = self.height_for_camera_target
        root_V_camera_goal_axis = map_P_human - root_P_camera
        root_V_camera_goal_axis.scale(1)
        root_V_camera_goal_axis_proj = cas.project_to_cone(root_V_camera_axis, root_V_camera_goal_axis, cone_theta)
        root_V_camera_goal_axis_proj.scale(1)

        angle = cas.angle_between_vector(root_V_camera_axis, root_V_camera_goal_axis)
        root_V_camera_goal_axis_proj = cas.if_less_eq(angle, np.pi / 1.9,
                                                      if_result=root_V_camera_goal_axis_proj,
                                                      else_result=root_V_goal_axis)

        root_V_camera_goal_axis_proj.vis_frame = self.camera_link
        god_map.debug_expression_manager.add_debug_expression('cone_axis',
                                                              root_V_camera_goal_axis,
                                                              color=ColorRGBA(r=1, g=1, b=0, a=1))
        god_map.debug_expression_manager.add_debug_expression('projected_axis',
                                                              root_V_camera_goal_axis_proj,
                                                              color=ColorRGBA(r=1, g=0.5, b=0, a=1))

        look_at_target = Task(name='look at target')
        self.add_task(look_at_target)
        if not self.drive_back:
            look_at_target.pause_condition = target_lost
        # god_map.debug_expression_manager.add_debug_expression('human', map_P_human)
        # god_map.debug_expression_manager.add_debug_expression('root_P_camera', root_P_camera)
        look_at_target.add_vector_goal_constraints(frame_V_current=root_V_camera_axis,
                                                   frame_V_goal=root_V_camera_goal_axis_proj,
                                                   reference_velocity=self.max_rotation_velocity_head,
                                                   weight=self.weight,
                                                   name='move camera')

        if not self.drive_back:
            arrived_at_target = Monitor(name='arrived at target')
            self.add_monitor(arrived_at_target)
            arrived_at_target.observation_expression = cas.less_equal(distance_to_human, self.min_distance_to_target)


        # TODO: if everything VFH related works, test deleting this
        # %% follow next point --tbr
        # follow_next_point = Task(name='follow next')
        # self.add_task(follow_next_point)
        # root_V_camera_axis.vis_frame = self.camera_link
        # root_V_camera_goal_axis.vis_frame = self.camera_link
        #
        # laser_violated = Monitor(name='laser violated')
        # self.add_monitor(laser_violated)
        # laser_violated.observation_expression = cas.less(closest_laser_reading, 0)
        # if self.drive_back:
        #     oriented_towards_next = Monitor(name='oriented towards next')
        #     oriented_towards_next.observation_expression = cas.abs(map_angle_error) > self.base_orientation_threshold
        #     self.add_monitor(oriented_towards_next)
        #
        #     follow_next_point.pause_condition = f'{laser_violated.name} or {oriented_towards_next}'
        # else:
        #     target_too_close = Monitor(name='target close')
        #     self.add_monitor(target_too_close)
        #     target_too_close.observation_expression = cas.less_equal(distance_to_human, self.min_distance_to_target)
        #
        #     follow_next_point.pause_condition = f'{laser_violated.name} or (not {target_lost.name} and {target_too_close.name})'
        # follow_next_point.add_point_goal_constraints(frame_P_current=root_P_bf,
        #                                              frame_P_goal=root_P_goal_point,
        #                                              reference_velocity=self.max_translation_velocity,
        #                                              weight=self.weight,
        #                                              name='min dist to next')
        god_map.debug_expression_manager.add_debug_expression('root_P_goal_point_original', root_P_goal_point)

        # %% keep the closest point in footprint radius
        stay_in_circle = Task(name='in circle')
        self.add_task(stay_in_circle)
        buffer = self.traj_tracking_radius
        eps = 0.00001
        distance_to_closest_point = cas.norm(root_P_closest_point - root_P_bf + cas.Point3([eps, eps, 0]))
        god_map.debug_expression_manager.add_debug_expression('root_P_closest_point', root_P_closest_point)
        god_map.debug_expression_manager.add_debug_expression('root_P_bf', root_P_bf)
        god_map.debug_expression_manager.add_debug_expression('distance_to_closest_point', distance_to_closest_point)
        stay_in_circle.add_inequality_constraint(task_expression=distance_to_closest_point,
                                                 lower_error=-distance_to_closest_point - buffer,
                                                 upper_error=-distance_to_closest_point + buffer,
                                                 reference_velocity=self.max_translation_velocity,
                                                 weight=self.weight,
                                                 name='stay in circle')

        # %% laser avoidance
        # todo will be completely replaced with VFH soon
        print(f'goal angle:{root_V_goal_angle}')
        vfh_move_task = VFHMoveDir(self.root, self.tip, self.laser_frame, root_V_goal_angle, name="vfh_move_task")
        self.add_task(vfh_move_task)

        if not self.drive_back:
            vfh_move_task.pause_condition = arrived_at_target
        # if self.enable_laser_avoidance: --tbr
        #     laser_avoidance_task = Task(name='laser avoidance')
        #     self.add_task(laser_avoidance_task)
        #     sideways_vel = (closest_laser_left + closest_laser_right)
        #     bf_V_laser_avoidance_direction = cas.Vector3([0, sideways_vel, 0])
        #     map_V_laser_avoidance_direction = root_T_bf.dot(bf_V_laser_avoidance_direction)
        #     map_V_laser_avoidance_direction.vis_frame = god_map.world.search_for_link_name(self.laser_frame)
        #     god_map.debug_expression_manager.add_debug_expression('base_V_laser_avoidance_direction',
        #                                                           map_V_laser_avoidance_direction)
        #     odom_y_vel = self.odom_joint.y_vel.get_symbol(Derivatives.position)
        #
        #     # turns laser avoidance off when robot is too far from path
        #     active = Monitor(name='too far from path')
        #     self.add_monitor(active)
        #     active.observation_expression = cas.greater(distance_to_closest_point, self.traj_tracking_radius)
        #
        #     buffer = self.laser_avoidance_sideways_buffer / 2
        #
        #     laser_avoidance_task.pause_condition = active.name
        #     #laser_avoidance_task.add_equality_constraint()
        #     laser_avoidance_task.add_inequality_constraint(reference_velocity=self.max_translation_velocity,
        #                                                    lower_error=sideways_vel - buffer,
        #                                                    upper_error=sideways_vel + buffer,
        #                                                    weight=WEIGHT_COLLISION_AVOIDANCE,
        #                                                    task_expression=odom_y_vel,
        #                                                    name='move sideways')

        if self.drive_back:
            first_traj_x = symbol_manager.get_symbol(self.ref_str + '.get_first_traj_point()[0]')
            first_traj_y = symbol_manager.get_symbol(self.ref_str + '.get_first_traj_point()[1]')
            first_point = cas.Point3([first_traj_x, first_traj_y, 0])
            goal_reached = Monitor(name='goal reached?')
            self.add_monitor(goal_reached)
            goal_reached.observation_expression = cas.euclidean_distance(first_point,
                                                                         root_P_bf) < self.traj_tracking_radius
            self.connect_end_condition_to_all_tasks(goal_reached.name)
            end = EndMotion(name='done')
            end.start_condition = goal_reached.name
            self.add_monitor(end)

    def clean_up(self):
        if CarryMyBullshit.target_sub is not None:
            CarryMyBullshit.target_sub.unregister()
            CarryMyBullshit.target_sub = None
        if CarryMyBullshit.laser_sub is not None:
            CarryMyBullshit.laser_sub.unregister()
            CarryMyBullshit.laser_sub = None
        if CarryMyBullshit.point_cloud_laser_sub is not None:
            CarryMyBullshit.point_cloud_laser_sub.unregister()
            CarryMyBullshit.point_cloud_laser_sub = None

    def init_laser_stuff(self, laser_scan: LaserScan):
        thresholds = []
        if len(laser_scan.ranges) % 2 == 0:
            print('laser range is even')
            angles = np.arange(laser_scan.angle_min,
                               laser_scan.angle_max,
                               laser_scan.angle_increment)[:-1]
        else:
            angles = np.arange(laser_scan.angle_min,
                               laser_scan.angle_max,
                               laser_scan.angle_increment)
        for angle in angles:
            if angle < 0:
                y = -self.laser_distance_threshold_width
                length = y / np.sin((angle))
                x = np.cos(angle) * length
                thresholds.append((x, y, length, angle))
            else:
                y = self.laser_distance_threshold_width
                length = y / np.sin((angle))
                x = np.cos(angle) * length
                thresholds.append((x, y, length, angle))
            if length > self.laser_distance_threshold:
                length = self.laser_distance_threshold
                x = np.cos(angle) * length
                y = np.sin(angle) * length
                thresholds[-1] = (x, y, length, angle)
        thresholds = np.array(thresholds)
        assert len(thresholds) == len(laser_scan.ranges)
        return thresholds

    def muddle_laser_scan(self, scan: LaserScan, thresholds: np.ndarray):
        data = np.array(scan.ranges)
        xs = np.cos(thresholds[:, 3]) * data
        ys = np.sin(thresholds[:, 3]) * data
        violations = data < thresholds[:, 2]
        xs_error = xs - thresholds[:, 0]
        half = int(data.shape[0] / 2)
        # x_positive = np.where(thresholds[:, 0] > 0)[0]
        x_below_laser_avoidance_threshold1 = thresholds[:, -1] > self.laser_avoidance_angle_cutout
        x_below_laser_avoidance_threshold2 = thresholds[:, -1] < -self.laser_avoidance_angle_cutout
        x_below_laser_avoidance_threshold = x_below_laser_avoidance_threshold1 | x_below_laser_avoidance_threshold2
        y_filter = x_below_laser_avoidance_threshold & violations
        closest_laser_right = ys[:half][y_filter[:half]]
        closest_laser_left = ys[half:][y_filter[half:]]

        x_positive = np.where(np.invert(x_below_laser_avoidance_threshold))[0]
        x_start = x_positive[0]
        x_end = x_positive[-1]

        front_violation = xs_error[x_start:x_end][violations[x_start:x_end]]
        if len(closest_laser_left) > 0:
            closest_laser_left = min(closest_laser_left)
        else:
            closest_laser_left = self.laser_distance_threshold_width
        if len(closest_laser_right) > 0:
            closest_laser_right = max(closest_laser_right)
        else:
            closest_laser_right = -self.laser_distance_threshold_width
        if len(front_violation) > 0:
            closest_laser_reading = min(front_violation)
        else:
            closest_laser_reading = 0
        return closest_laser_reading, closest_laser_left, closest_laser_right

    def laser_cb(self, scan: LaserScan):
        self.last_scan = scan
        if self.thresholds is None:
            self.thresholds = self.init_laser_stuff(scan)
            self.publish_laser_thresholds()
        self.closest_laser_reading, self.closest_laser_left, self.closest_laser_right = self.muddle_laser_scan(scan,
                                                                                                               self.thresholds)
        self.get_current_target()
        # print(f'TAR:{self.current_target}')

        self.vfh.run(target_point=self.current_target[:3].copy())

    def point_cloud_laser_cb(self, scan: LaserScan):
        self.last_scan_pc = scan
        if self.thresholds_pc is None:
            self.thresholds_pc = self.init_laser_stuff(scan)
        self.closest_laser_reading_pc, self.closest_laser_left_pc, self.closest_laser_right_pc = self.muddle_laser_scan(
            scan, self.thresholds_pc)

    def get_current_point(self) -> np.ndarray:
        root_T_tip = god_map.world.compute_fk_np(self.root, self.tip)
        x = root_T_tip[0, 3]
        y = root_T_tip[1, 3]
        return np.array([x, y])

    def get_first_traj_point(self) -> np.ndarray:
        return CarryMyBullshit.traj_data[0]

    # @memoize_with_counter(4)
    def get_current_target(self) -> Dict[str, float]:
        self.check_laser_scan_age()
        traj = CarryMyBullshit.trajectory.copy()
        current_point = self.get_current_point()
        error = traj - current_point
        distances = np.linalg.norm(error, axis=1)
        # cut off old points
        in_radius = np.where(distances < self.traj_tracking_radius)[0]
        if len(in_radius) > 0:
            next_idx = max(in_radius)
            offset = max(0, next_idx - self.max_temp_distance)
            closest_idx = np.argmin(distances[offset:]) + offset
        else:
            next_idx = closest_idx = np.argmin(distances)
        # CarryMyBullshit.traj_data = CarryMyBullshit.traj_data[closest_idx:]
        if not self.drive_back:
            self.last_target_age = rospy.get_rostime().to_sec() - self.human_point.header.stamp.to_sec()
            if self.last_target_age > self.target_age_exception_threshold:
                raise_to_blackboard(
                    ExecutionException(f'lost target for longer than {self.target_age_exception_threshold}s'))
        else:
            if closest_idx == CarryMyBullshit.trajectory.shape[0] - 1:
                self.end_of_traj_reached = True
        result = {
            'next_x': traj[next_idx, 0],
            'next_y': traj[next_idx, 1],
            'closest_x': traj[closest_idx, 0],
            'closest_y': traj[closest_idx, 1],
        }

        self.current_target = np.array([traj[next_idx, 0], traj[next_idx, 1], 0, 1])
        laser_T_root = god_map.world.compute_fk_np(self.laser_frame, self.root)
        self.current_target = laser_T_root @ self.current_target

        self.closest_point = np.array([traj[closest_idx, 0], traj[closest_idx, 1], 0, 1])
        return result

    def check_laser_scan_age(self):
        current_time = rospy.get_rostime().to_sec()
        base_laser_age = current_time - self.last_scan.header.stamp.to_sec()
        if base_laser_age > self.laser_scan_age_threshold:
            get_middleware().logwarn(f'last base laser scan is too old: {base_laser_age}')
            self.closest_laser_left = self.laser_distance_threshold_width
            self.closest_laser_right = -self.laser_distance_threshold_width
            self.closest_laser_reading = 0
        point_cloud_laser_age = current_time - self.last_scan_pc.header.stamp.to_sec()
        if point_cloud_laser_age > self.laser_scan_age_threshold and CarryMyBullshit.point_cloud_laser_sub is not None:
            get_middleware().logwarn(f'last point cloud laser scan is too old: {point_cloud_laser_age}')
            self.closest_laser_left_pc = self.laser_distance_threshold_width
            self.closest_laser_right_pc = -self.laser_distance_threshold_width
            self.closest_laser_reading_pc = 0

    def is_done(self):
        return self.end_of_traj_reached

    def publish_trajectory(self):
        ms = MarkerArray()
        m_line = Marker()
        m_line.action = m_line.ADD
        m_line.ns = 'traj'
        m_line.id = 1
        m_line.type = m_line.LINE_STRIP
        m_line.header.frame_id = str(god_map.world.root_link_name)
        m_line.scale.x = 0.05
        m_line.color.a = 1
        m_line.color.r = 1
        try:
            for item in CarryMyBullshit.trajectory:
                p = Point()
                p.x = item[0]
                p.y = item[1]
                m_line.points.append(p)
            ms.markers.append(m_line)
        except Exception as e:
            get_middleware().logwarn('failed to create traj marker')
        self.pub.publish(ms)

    def publish_laser_thresholds(self):
        ms = MarkerArray()
        m_line = Marker()
        m_line.action = m_line.ADD
        m_line.ns = 'laser_thresholds'
        m_line.id = 1332
        m_line.type = m_line.LINE_STRIP
        m_line.header.frame_id = self.laser_frame
        m_line.scale.x = 0.05
        m_line.color.a = 1
        m_line.color.r = 0.5
        m_line.color.b = 1
        m_line.frame_locked = True
        for item in self.thresholds:
            p = Point()
            p.x = item[0]
            p.y = item[1]
            m_line.points.append(p)
        ms.markers.append(m_line)
        square = Marker()
        square.action = m_line.ADD
        square.ns = 'laser_avoidance_angle_cutout'
        square.id = 1333
        square.type = m_line.LINE_STRIP
        square.header.frame_id = self.laser_frame
        # p = Point()
        # p.x = self.thresholds[0, 0]
        # p.y = self.thresholds[0, 1]
        # square.points.append(p)
        p = Point()
        idx = np.where(self.thresholds[:, -1] < -self.laser_avoidance_angle_cutout)[0][-1]
        p.x = self.thresholds[idx, 0]
        p.y = self.thresholds[idx, 1]
        square.points.append(p)
        p = Point()
        square.points.append(p)
        p = Point()
        idx = np.where(self.thresholds[:, -1] > self.laser_avoidance_angle_cutout)[0][0]
        p.x = self.thresholds[idx, 0]
        p.y = self.thresholds[idx, 1]
        square.points.append(p)
        # p = Point()
        # p.x = self.thresholds[-1, 0]
        # p.y = self.thresholds[-1, 1]
        # square.points.append(p)
        square.scale.x = 0.05
        square.color.a = 1
        square.color.r = 0.5
        square.color.b = 1
        square.frame_locked = True
        ms.markers.append(square)
        self.pub.publish(ms)

    def publish_tracking_radius(self):
        ms = MarkerArray()
        m_line = Marker()
        m_line.action = m_line.ADD
        m_line.ns = 'traj_tracking_radius'
        m_line.id = 132
        m_line.type = m_line.CYLINDER
        m_line.header.frame_id = str(self.tip.short_name)
        m_line.scale.x = self.traj_tracking_radius * 2
        m_line.scale.y = self.traj_tracking_radius * 2
        m_line.scale.z = 0.05
        m_line.color.a = 0.5
        m_line.color.b = 1
        m_line.pose.orientation.w = 1
        m_line.frame_locked = True
        ms.markers.append(m_line)
        self.pub.publish(ms)

    def publish_distance_to_target(self):
        ms = MarkerArray()
        m_line = Marker()
        m_line.action = m_line.ADD
        m_line.ns = 'distance_to_target_stop_threshold'
        m_line.id = 133
        m_line.type = m_line.CYLINDER
        m_line.header.frame_id = str(self.tip.short_name)
        m_line.scale.x = self.min_distance_to_target * 2
        m_line.scale.y = self.min_distance_to_target * 2
        m_line.scale.z = 0.01
        m_line.color.a = 0.5
        m_line.color.g = 1
        m_line.pose.orientation.w = 1
        m_line.frame_locked = True
        ms.markers.append(m_line)
        self.pub.publish(ms)

    def target_cb(self, point: PointStamped):
        try:
            current_point = np.array([point.point.x, point.point.y])
            last_point = CarryMyBullshit.traj_data[-1]
            error_vector = current_point - last_point
            distance = np.linalg.norm(error_vector)
            if distance < self.interpolation_step_size * 2:
                CarryMyBullshit.traj_data[-1] = 0.5 * CarryMyBullshit.traj_data[-1] + 0.5 * current_point
            else:
                error_vector /= distance
                ranges = np.arange(self.interpolation_step_size, distance, self.interpolation_step_size)
                interpolated_distance = distance / len(ranges)
                for i, dt in enumerate(ranges):
                    interpolated_point = last_point + error_vector * interpolated_distance * (i + 1)
                    CarryMyBullshit.traj_data.append(interpolated_point)
                CarryMyBullshit.traj_data.append(current_point)

            CarryMyBullshit.trajectory = np.array(CarryMyBullshit.traj_data)
            self.human_point.point.x = CarryMyBullshit.traj_data[-1][0]
            self.human_point.point.y = CarryMyBullshit.traj_data[-1][1]
            self.human_point = point
        except Exception as e:
            get_middleware().logwarn(f'rejected new target because: {e}')
        self.publish_trajectory()


class FollowNavPath(Goal):
    pass
