import os
from copy import deepcopy
from typing import Dict

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PointStamped, Vector3Stamped, Pose
from numpy import pi
from tf.transformations import quaternion_from_matrix, quaternion_about_axis

from giskardpy.data_types.exceptions import EmptyProblemException
from giskardpy.data_types.exceptions import ObjectForceTorqueThresholdException
from giskardpy.data_types.suturo_types import ForceTorqueThresholds
from giskardpy.data_types.suturo_types import GraspTypes
from giskardpy.god_map import god_map
from giskardpy.motion_graph.monitors.lidar_monitor import LidarPayloadMonitor
from giskardpy.motion_graph.tasks.task import WEIGHT_ABOVE_CA
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.hsr import HSRCollisionAvoidanceConfig, WorldWithHSRConfig, HSRStandaloneInterface
from utils_for_tests import compare_poses, GiskardTestWrapper
from utils_for_tests import launch_launchfile

if 'GITHUB_WORKFLOW' not in os.environ:
    from giskardpy.goals.suturo import Reaching, TakePose, VerticalMotion, AlignHeight, Tilting, Placing


class HSRTestWrapper(GiskardTestWrapper):
    default_pose = {
        'arm_flex_joint': -0.03,
        'arm_lift_joint': 0.01,
        'arm_roll_joint': 0.0,
        'head_pan_joint': 0.0,
        'head_tilt_joint': 0.0,
        'wrist_flex_joint': 0.0,
        'wrist_roll_joint': 0.0,
    }
    better_pose = default_pose

    def __init__(self, giskard=None):
        self.tip = 'hand_gripper_tool_frame'
        if giskard is None:
            giskard = Giskard(world_config=WorldWithHSRConfig(),
                              collision_avoidance_config=HSRCollisionAvoidanceConfig(),
                              robot_interface_config=HSRStandaloneInterface(),
                              behavior_tree_config=StandAloneBTConfig(debug_mode=True,
                                                                      publish_tf=True,
                                                                      publish_js=False),
                              qp_controller_config=QPControllerConfig(mpc_dt=0.05))
        super().__init__(giskard)
        self.gripper_group = 'gripper'
        # self.r_gripper = rospy.ServiceProxy('r_gripper_simulator/set_joint_states', SetJointState)
        # self.l_gripper = rospy.ServiceProxy('l_gripper_simulator/set_joint_states', SetJointState)
        self.odom_root = 'odom'
        self.robot = god_map.world.groups[self.robot_name]

    def open_gripper(self):
        self.command_gripper(1.23)

    def close_gripper(self):
        self.command_gripper(0)

    def command_gripper(self, width):
        js = {'hand_motor_joint': width}
        self.set_joint_goal(js)
        self.allow_all_collisions()
        self.execute()

    def reset(self):
        pass


@pytest.fixture(scope='module')
def giskard(request, ros):
    launch_launchfile('package://hsr_description/launch/upload_hsrb.launch')
    c = HSRTestWrapper()
    # c = HSRTestWrapperMujoco()
    request.addfinalizer(c.tear_down)
    return c


@pytest.fixture()
def box_setup(zero_pose: HSRTestWrapper) -> HSRTestWrapper:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = 1.2
    p.pose.position.y = 0
    p.pose.position.z = 0.1
    p.pose.orientation.w = 1
    zero_pose.add_box_to_world(name='box', size=(1, 1, 1), pose=p)
    return zero_pose


# TODO: Further rework force Monitor test; removing unnecessary Code, create more Tests etc.
# FIXME: Tests don't work with the new changes
# class TestForceMonitor:
#     """
#     The tests for the force_monitor require rosbags which have been recorded on the
#     /hsrb/wrist_wrench/compensated topic. Since there's no other way to properly
#     simulate/imitate the forces produced by the force-torque sensor.
#     """
#
#     def test_force_monitor_grasp(self, zero_pose: HSRTestWrapper):
#         sleep = zero_pose.monitors.add_sleep(2.5)
#         force_torque = zero_pose.monitors.add_monitor(monitor_class=PayloadForceTorque.__name__,
#                                                       name=PayloadForceTorque.__name__,
#                                                       start_condition='',
#                                                       threshold_name=ForceTorqueThresholds.FT_GraspWithCare.value,
#                                                       is_raw=False,
#                                                       object_type=ObjectTypes.OT_Standard.value)
#
#         base_goal = PoseStamped()
#         base_goal.header.frame_id = 'map'
#         base_goal.pose.position.x = 1
#         base_goal.pose.orientation.w = 1
#         goal_reached = zero_pose.monitors.add_cartesian_pose(goal_pose=base_goal,
#                                                              tip_link='base_footprint',
#                                                              root_link='map',
#                                                              name='goal reached')
#
#         zero_pose.motion_goals.add_cartesian_pose(goal_pose=base_goal,
#                                                   tip_link='base_footprint',
#                                                   root_link='map',
#                                                   hold_condition=force_torque,
#                                                   end_condition=f'{goal_reached} and {sleep}')
#         local_min = zero_pose.monitors.add_local_minimum_reached(start_condition=goal_reached)
#
#         zero_pose.monitors.add_end_motion(start_condition=f'{local_min} and {sleep}')
#         zero_pose.motion_goals.allow_all_collisions()
#         zero_pose.set_max_traj_length(100)
#         zero_pose.execute(add_local_minimum_reached=False)
#
#     def test_force_monitor_placing(self, zero_pose: HSRTestWrapper):
#         sleep = zero_pose.monitors.add_sleep(2.5)
#         force_torque = zero_pose.monitors.add_monitor(monitor_class=PayloadForceTorque.__name__,
#                                                       name=PayloadForceTorque.__name__,
#                                                       start_condition='',
#                                                       threshold_name=ForceTorqueThresholds.FT_Placing.value,
#                                                       is_raw=False,
#                                                       object_type=ObjectTypes.OT_Standard.value)
#
#         base_goal = PoseStamped()
#         base_goal.header.frame_id = 'map'
#         base_goal.pose.position.x = 1
#         base_goal.pose.orientation.w = 1
#         goal_reached = zero_pose.monitors.add_cartesian_pose(goal_pose=base_goal,
#                                                              tip_link='base_footprint',
#                                                              root_link='map',
#                                                              name='goal reached')
#
#         zero_pose.motion_goals.add_cartesian_pose(goal_pose=base_goal,
#                                                   tip_link='base_footprint',
#                                                   root_link='map',
#                                                   hold_condition=force_torque,
#                                                   end_condition=f'{goal_reached} and {sleep}')
#         local_min = zero_pose.monitors.add_local_minimum_reached(start_condition=goal_reached)
#
#         zero_pose.monitors.add_end_motion(start_condition=f'{local_min} and {sleep}')
#         zero_pose.motion_goals.allow_all_collisions()
#         zero_pose.set_max_traj_length(100)
#         zero_pose.execute(add_local_minimum_reached=False)


class TestLidarMonitor:

    # Zur Zeit kein automatisch ausführbarer Test
    def test_lidar_monitor(self, zero_pose: HSRTestWrapper):
        lidar = zero_pose.monitors.add_monitor(monitor_class=LidarPayloadMonitor.__name__,
                                               name=LidarPayloadMonitor.__name__ + 'Test',
                                               start_condition='',
                                               topic='/hokuyo_back/most_intense',
                                               frame_id='laser_reference_back',
                                               laser_distance_threshold_width=0.5,
                                               laser_distance_threshold=0.8)

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.orientation.w = 1
        goal_reached = zero_pose.monitors.add_cartesian_pose(goal_pose=base_goal,
                                                             tip_link='base_footprint',
                                                             root_link='map',
                                                             name='goal reached')

        zero_pose.motion_goals.add_cartesian_pose(goal_pose=base_goal,
                                                  tip_link='base_footprint',
                                                  root_link='map',
                                                  hold_condition=lidar,
                                                  end_condition=f'{goal_reached}')

        local_min = zero_pose.monitors.add_local_minimum_reached(start_condition=goal_reached)

        zero_pose.monitors.add_end_motion(start_condition=f'{local_min}')
        zero_pose.motion_goals.allow_all_collisions()
        zero_pose.execute(add_local_minimum_reached=False)


class TestJointGoals:

    def test_mimic_joints(self, zero_pose: HSRTestWrapper):
        arm_lift_joint = god_map.world.search_for_joint_name('arm_lift_joint')
        zero_pose.open_gripper()
        hand_T_finger_current = zero_pose.compute_fk_pose('hand_palm_link', 'hand_l_distal_link')
        hand_T_finger_expected = PoseStamped()
        hand_T_finger_expected.header.frame_id = 'hand_palm_link'
        hand_T_finger_expected.pose.position.x = -0.01675
        hand_T_finger_expected.pose.position.y = -0.0907
        hand_T_finger_expected.pose.position.z = 0.0052
        hand_T_finger_expected.pose.orientation.x = -0.0434
        hand_T_finger_expected.pose.orientation.y = 0.0
        hand_T_finger_expected.pose.orientation.z = 0.0
        hand_T_finger_expected.pose.orientation.w = 0.999
        compare_poses(hand_T_finger_current.pose, hand_T_finger_expected.pose)

        js = {'torso_lift_joint': 0.1}
        zero_pose.set_joint_goal(js, add_monitor=False)
        zero_pose.allow_all_collisions()
        zero_pose.execute()
        np.testing.assert_almost_equal(god_map.world.state[arm_lift_joint].position, 0.2, decimal=2)
        base_T_torso = PoseStamped()
        base_T_torso.header.frame_id = 'base_footprint'
        base_T_torso.pose.position.x = 0
        base_T_torso.pose.position.y = 0
        base_T_torso.pose.position.z = 0.8518
        base_T_torso.pose.orientation.x = 0
        base_T_torso.pose.orientation.y = 0
        base_T_torso.pose.orientation.z = 0
        base_T_torso.pose.orientation.w = 1
        base_T_torso2 = zero_pose.compute_fk_pose('base_footprint', 'torso_lift_link')
        compare_poses(base_T_torso2.pose, base_T_torso.pose)

        zero_pose.close_gripper()

    def test_mimic_joints2(self, zero_pose: HSRTestWrapper):
        arm_lift_joint = god_map.world.search_for_joint_name('arm_lift_joint')
        zero_pose.open_gripper()

        tip = 'hand_gripper_tool_frame'
        p = PoseStamped()
        p.header.frame_id = tip
        p.pose.position.z = 0.2
        p.pose.orientation.w = 1
        zero_pose.set_cart_goal(goal_pose=p, tip_link=tip,
                                root_link='base_footprint')
        zero_pose.allow_all_collisions()
        zero_pose.execute()
        np.testing.assert_almost_equal(god_map.world.state[arm_lift_joint].position, 0.2, decimal=2)
        base_T_torso = PoseStamped()
        base_T_torso.header.frame_id = 'base_footprint'
        base_T_torso.pose.position.x = 0
        base_T_torso.pose.position.y = 0
        base_T_torso.pose.position.z = 0.8518
        base_T_torso.pose.orientation.x = 0
        base_T_torso.pose.orientation.y = 0
        base_T_torso.pose.orientation.z = 0
        base_T_torso.pose.orientation.w = 1
        base_T_torso2 = zero_pose.compute_fk_pose('base_footprint', 'torso_lift_link')
        compare_poses(base_T_torso2.pose, base_T_torso.pose)

        zero_pose.close_gripper()

    def test_mimic_joints3(self, zero_pose: HSRTestWrapper):
        arm_lift_joint = god_map.world.search_for_joint_name('arm_lift_joint')
        zero_pose.open_gripper()
        tip = 'head_pan_link'
        p = PoseStamped()
        p.header.frame_id = tip
        p.pose.position.z = 0.15
        p.pose.orientation.w = 1
        zero_pose.set_cart_goal(goal_pose=p, tip_link=tip,
                                root_link='base_footprint')
        zero_pose.execute()
        np.testing.assert_almost_equal(god_map.world.state[arm_lift_joint].position, 0.3, decimal=2)
        base_T_torso = PoseStamped()
        base_T_torso.header.frame_id = 'base_footprint'
        base_T_torso.pose.position.x = 0
        base_T_torso.pose.position.y = 0
        base_T_torso.pose.position.z = 0.902
        base_T_torso.pose.orientation.x = 0
        base_T_torso.pose.orientation.y = 0
        base_T_torso.pose.orientation.z = 0
        base_T_torso.pose.orientation.w = 1
        base_T_torso2 = zero_pose.compute_fk_pose('base_footprint', 'torso_lift_link')
        compare_poses(base_T_torso2.pose, base_T_torso.pose)

        zero_pose.close_gripper()

    def test_mimic_joints4(self, zero_pose: HSRTestWrapper):
        ll, ul = god_map.world.get_joint_velocity_limits('hsrb/arm_lift_joint')
        assert ll == -0.15
        assert ul == 0.15
        ll, ul = god_map.world.get_joint_velocity_limits('hsrb/torso_lift_joint')
        assert ll == -0.075
        assert ul == 0.075
        joint_goal = {'torso_lift_joint': 0.25}
        zero_pose.set_joint_goal(joint_goal, add_monitor=False)
        zero_pose.allow_all_collisions()
        zero_pose.execute()
        np.testing.assert_almost_equal(god_map.world.state['hsrb/arm_lift_joint'].position, 0.5, decimal=2)


class TestCartGoals:
    def test_save_graph_pdf(self, kitchen_setup):
        box1_name = 'box1'
        pose = PoseStamped()
        pose.header.frame_id = kitchen_setup.default_root
        pose.pose.orientation.w = 1
        kitchen_setup.add_box_to_world(name=box1_name,
                                       size=(1, 1, 1),
                                       pose=pose,
                                       parent_link='hand_palm_link')
        god_map.world.save_graph_pdf(god_map.tmp_folder)

    def test_move_base(self, zero_pose: HSRTestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.position.x = 1
        map_T_odom.pose.position.y = 1
        map_T_odom.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 3, [0, 0, 1]))
        zero_pose.teleport_base(map_T_odom)

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi, [0, 0, 1]))
        zero_pose.set_cart_goal(goal_pose=base_goal, tip_link='base_footprint', root_link='map')
        zero_pose.allow_all_collisions()
        zero_pose.execute()

    def test_move_base_1m_forward(self, zero_pose: HSRTestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.position.x = 1
        map_T_odom.pose.orientation.w = 1
        zero_pose.allow_all_collisions()
        zero_pose.move_base(map_T_odom)

    def test_move_base_1m_left(self, zero_pose: HSRTestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.position.y = 1
        map_T_odom.pose.orientation.w = 1
        zero_pose.allow_all_collisions()
        zero_pose.move_base(map_T_odom)

    def test_move_base_1m_diagonal(self, zero_pose: HSRTestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.position.x = 1
        map_T_odom.pose.position.y = 1
        map_T_odom.pose.orientation.w = 1
        zero_pose.allow_all_collisions()
        zero_pose.move_base(map_T_odom)

    def test_move_base_rotate(self, zero_pose: HSRTestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 3, [0, 0, 1]))
        zero_pose.allow_all_collisions()
        zero_pose.move_base(map_T_odom)

    def test_move_base_forward_rotate(self, zero_pose: HSRTestWrapper):
        map_T_odom = PoseStamped()
        map_T_odom.header.frame_id = 'map'
        map_T_odom.pose.position.x = 1
        map_T_odom.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 3, [0, 0, 1]))
        zero_pose.allow_all_collisions()
        zero_pose.move_base(map_T_odom)

    def test_rotate_gripper(self, zero_pose: HSRTestWrapper):
        r_goal = PoseStamped()
        r_goal.header.frame_id = zero_pose.tip
        r_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi, [0, 0, 1]))
        zero_pose.set_cart_goal(goal_pose=r_goal, tip_link=zero_pose.tip, root_link='map')
        zero_pose.allow_all_collisions()
        zero_pose.execute()


class TestConstraints:

    def test_open_fridge(self, kitchen_setup: HSRTestWrapper):
        handle_frame_id = 'iai_kitchen/iai_fridge_door_handle'
        handle_name = 'iai_fridge_door_handle'
        kitchen_setup.open_gripper()
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.3, -0.5, 0)
        base_goal.pose.orientation.w = 1
        kitchen_setup.move_base(base_goal)

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.z = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.tip
        tip_grasp_axis.vector.x = 1

        kitchen_setup.set_grasp_bar_goal(root_link=kitchen_setup.default_root,
                                         tip_link=kitchen_setup.tip,
                                         tip_grasp_axis=tip_grasp_axis,
                                         bar_center=bar_center,
                                         bar_axis=bar_axis,
                                         bar_length=.4)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        kitchen_setup.set_align_planes_goal(tip_link=kitchen_setup.tip,
                                            tip_normal=x_gripper,
                                            goal_normal=x_goal,
                                            root_link='map')
        kitchen_setup.allow_all_collisions()
        # kitchen_setup.add_json_goal('AvoidJointLimits', percentage=10)
        kitchen_setup.execute()

        kitchen_setup.close_gripper()

        current_pose = kitchen_setup.compute_fk_pose(root_link='map', tip_link=kitchen_setup.tip)

        kitchen_setup.set_open_container_goal(tip_link=kitchen_setup.tip,
                                              environment_link=handle_name,
                                              goal_joint_state=1.5)

        kitchen_setup.allow_all_collisions()
        # kitchen_setup.add_json_goal('AvoidJointLimits')
        kitchen_setup.execute()
        kitchen_setup.set_env_state({'iai_fridge_door_joint': 1.5})

        pose_reached = kitchen_setup.monitors.add_cartesian_pose('map',
                                                                 tip_link=kitchen_setup.tip,
                                                                 goal_pose=current_pose)
        kitchen_setup.monitors.add_end_motion(start_condition=pose_reached)

        kitchen_setup.set_open_container_goal(tip_link=kitchen_setup.tip,
                                              environment_link=handle_name,
                                              goal_joint_state=0)
        kitchen_setup.allow_all_collisions()

        kitchen_setup.execute(add_local_minimum_reached=False)

        kitchen_setup.set_env_state({'iai_fridge_door_joint': 0})
        kitchen_setup.open_gripper()

        kitchen_setup.set_joint_goal(kitchen_setup.better_pose)
        kitchen_setup.allow_self_collision()
        kitchen_setup.execute()

        kitchen_setup.close_gripper()

    def test_open_dishwasher1(self, kitchen_setup: HSRTestWrapper):
        handle_frame_id = 'iai_kitchen/sink_area_dish_washer_door_handle'
        handle_name = 'sink_area_dish_washer_door_handle'
        kitchen_setup.open_gripper()
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.3, -0.3, 0)
        base_goal.pose.orientation.w = 1
        kitchen_setup.move_base(base_goal)

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.tip
        tip_grasp_axis.vector.x = 1

        kitchen_setup.set_grasp_bar_goal(root_link=kitchen_setup.default_root,
                                         tip_link=kitchen_setup.tip,
                                         tip_grasp_axis=tip_grasp_axis,
                                         bar_center=bar_center,
                                         bar_axis=bar_axis,
                                         bar_length=.4)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        kitchen_setup.set_align_planes_goal(tip_link=kitchen_setup.tip,
                                            tip_normal=x_gripper,
                                            goal_normal=x_goal,
                                            root_link='map')
        kitchen_setup.allow_all_collisions()
        kitchen_setup.execute()

        kitchen_setup.close_gripper()

        current_pose = kitchen_setup.compute_fk_pose(root_link='map', tip_link=kitchen_setup.tip)

        kitchen_setup.set_open_container_goal(tip_link=kitchen_setup.tip,
                                              environment_link=handle_name,
                                              goal_joint_state=1.5)

        kitchen_setup.allow_all_collisions()
        kitchen_setup.execute()

        pose_reached = kitchen_setup.monitors.add_cartesian_pose('map',
                                                                 tip_link=kitchen_setup.tip,
                                                                 goal_pose=current_pose)
        kitchen_setup.monitors.add_end_motion(start_condition=pose_reached)

        kitchen_setup.set_close_container_goal(tip_link=kitchen_setup.tip,
                                               environment_link=handle_name)
        kitchen_setup.allow_all_collisions()

        kitchen_setup.execute(add_local_minimum_reached=False)

        kitchen_setup.open_gripper()

        kitchen_setup.set_joint_goal(kitchen_setup.better_pose)
        kitchen_setup.allow_self_collision()
        kitchen_setup.execute()

    def test_open_fridge_sequence(self, kitchen_setup: HSRTestWrapper):
        handle_frame_id = 'iai_kitchen/iai_fridge_door_handle'
        handle_name = 'iai_fridge_door_handle'
        kitchen_setup.open_gripper()
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.3, -0.5, 0)
        base_goal.pose.orientation.w = 1
        kitchen_setup.allow_all_collisions()
        kitchen_setup.move_base(base_goal)

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.z = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.tip
        tip_grasp_axis.vector.x = 1

        # %% phase 1
        bar_grasped = kitchen_setup.monitors.add_distance_to_line(name='bar grasped',
                                                                  root_link=kitchen_setup.default_root,
                                                                  tip_link=kitchen_setup.tip,
                                                                  center_point=bar_center,
                                                                  line_axis=bar_axis,
                                                                  line_length=.4)
        kitchen_setup.motion_goals.add_grasp_bar(root_link=kitchen_setup.default_root,
                                                 tip_link=kitchen_setup.tip,
                                                 tip_grasp_axis=tip_grasp_axis,
                                                 bar_center=bar_center,
                                                 bar_axis=bar_axis,
                                                 bar_length=.4,
                                                 name='grasp bar',
                                                 end_condition=bar_grasped)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        kitchen_setup.motion_goals.add_align_planes(tip_link=kitchen_setup.tip,
                                                    tip_normal=x_gripper,
                                                    goal_normal=x_goal,
                                                    root_link='map',
                                                    name='orient to door',
                                                    end_condition=bar_grasped)

        # %% phase 2 open door
        door_open = kitchen_setup.monitors.add_local_minimum_reached(name='door open',
                                                                     start_condition=bar_grasped)
        kitchen_setup.motion_goals.add_open_container(tip_link=kitchen_setup.tip,
                                                      environment_link=handle_name,
                                                      goal_joint_state=1.5,
                                                      name='open door',
                                                      start_condition=bar_grasped,
                                                      end_condition=door_open)

        kitchen_setup.allow_all_collisions()
        kitchen_setup.monitors.add_end_motion(start_condition=door_open)
        kitchen_setup.execute(add_local_minimum_reached=False)

        kitchen_setup.close_gripper()

    def test_open_dishwasher2(self, kitchen_setup: HSRTestWrapper):
        handle_frame_id = 'iai_kitchen/sink_area_dish_washer_door_handle'
        handle_name = handle_frame_id
        hinge_joint = god_map.world.get_movable_parent_joint(handle_frame_id)
        door_hinge_frame_id = god_map.world.get_parent_link_of_link(handle_frame_id)

        print(door_hinge_frame_id)

        kitchen_setup.open_gripper()
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.3, -0.3, 0)
        base_goal.pose.orientation.w = 1
        kitchen_setup.move_base(base_goal)

        kitchen_setup.set_hsrb_dishwasher_door_handle_grasp(root_link=kitchen_setup.default_root,
                                                            tip_link=kitchen_setup.tip,
                                                            grasp_bar_offset=0.02,
                                                            handle_frame_id=handle_frame_id)

        kitchen_setup.allow_all_collisions()
        kitchen_setup.execute()

        kitchen_setup.close_gripper()

        kitchen_setup.motion_goals.add_open_container(tip_link=kitchen_setup.tip,
                                                      environment_link=handle_name,
                                                      goal_joint_state=1.5)

        kitchen_setup.allow_all_collisions()
        kitchen_setup.execute()

        kitchen_setup.open_gripper()

        kitchen_setup.motion_goals.hsrb_dishwasher_door_around(handle_name=handle_name,
                                                               root_link=kitchen_setup.default_root,
                                                               tip_link=kitchen_setup.tip)

        kitchen_setup.execute()

        kitchen_setup.motion_goals.hsrb_align_to_push_door_goal(root_link=kitchen_setup.default_root,
                                                                tip_link=kitchen_setup.tip,
                                                                handle_name=handle_name,
                                                                hinge_frame_id=door_hinge_frame_id)

        kitchen_setup.execute()

        kitchen_setup.close_gripper()

        kitchen_setup.motion_goals.hsrb_pre_push_door_goal(root_link=kitchen_setup.default_root,
                                                           tip_link=kitchen_setup.tip,
                                                           handle_name=handle_name,
                                                           hinge_frame_id=door_hinge_frame_id)

        kitchen_setup.allow_collision(kitchen_setup.default_env_name, kitchen_setup.gripper_group)
        kitchen_setup.execute()

        kitchen_setup.set_open_container_goal(tip_link=kitchen_setup.tip,
                                              environment_link=handle_name,
                                              goal_joint_state=1.5)

        kitchen_setup.allow_collision(kitchen_setup.default_env_name, kitchen_setup.robot_name)
        kitchen_setup.execute()

    def test_open_dishwasher3(self, kitchen_setup: HSRTestWrapper):
        if 'GITHUB_WORKFLOW' in os.environ:
            return
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.3, -0.3, 0)
        base_goal.pose.orientation.w = 1
        kitchen_setup.move_base(base_goal)

        handle_frame_id = 'iai_kitchen/sink_area_dish_washer_door_handle'
        handle_name = handle_frame_id
        hinge_joint = god_map.world.get_movable_parent_joint(handle_frame_id)
        door_hinge_frame_id = god_map.world.get_parent_link_of_link(handle_frame_id)
        root_link = kitchen_setup.default_root
        tip_link = kitchen_setup.tip
        grasp_bar_offset = 0.02
        goal_angle_half = 0.6
        goal_angle_full = 1.5
        env_name = 'iai_kitchen'
        gripper_group = 'gripper'

        first_open = kitchen_setup.monitors.add_open_hsr_gripper(name='first open')

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis_bar = Vector3Stamped()
        tip_grasp_axis_bar.header.frame_id = tip_link
        tip_grasp_axis_bar.vector.x = 1

        grasp_axis_offset = Vector3Stamped()
        grasp_axis_offset.header.frame_id = handle_frame_id
        grasp_axis_offset.vector.x = -grasp_bar_offset

        tip_grasp_axis_push = Vector3Stamped()
        tip_grasp_axis_push.header.frame_id = tip_link
        tip_grasp_axis_push.vector.y = 1

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = tip_link
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1

        bar_grasped = kitchen_setup.monitors.add_distance_to_line(name='bar grasped',
                                                                  root_link=root_link,
                                                                  tip_link=tip_link,
                                                                  center_point=bar_center,
                                                                  line_axis=bar_axis,
                                                                  line_length=.4)

        kitchen_setup.motion_goals.add_grasp_bar_offset(name='grasp bar',
                                                        root_link=root_link,
                                                        tip_link=tip_link,
                                                        tip_grasp_axis=tip_grasp_axis_bar,
                                                        bar_center=bar_center,
                                                        bar_axis=bar_axis,
                                                        bar_length=.4,
                                                        grasp_axis_offset=grasp_axis_offset,
                                                        start_condition=first_open,
                                                        end_condition=bar_grasped)

        kitchen_setup.motion_goals.add_align_planes(tip_link=tip_link,
                                                    tip_normal=x_gripper,
                                                    goal_normal=x_goal,
                                                    root_link=root_link,
                                                    start_condition=first_open,
                                                    end_condition=bar_grasped)

        first_close = kitchen_setup.monitors.add_close_hsr_gripper(name='first close', start_condition=bar_grasped)

        half_open_joint = kitchen_setup.monitors.add_joint_position(name='half open joint',
                                                                    goal_state={hinge_joint: goal_angle_half},
                                                                    threshold=0.02,
                                                                    start_condition=first_close)

        kitchen_setup.motion_goals.add_open_container(name='half open',
                                                      tip_link=tip_link,
                                                      environment_link=handle_name,
                                                      goal_joint_state=goal_angle_half,
                                                      start_condition=first_close,
                                                      end_condition=half_open_joint)

        final_open = kitchen_setup.monitors.add_open_hsr_gripper(name='final open', start_condition=half_open_joint)

        around_local_min = kitchen_setup.monitors.add_local_minimum_reached(name='around door local min',
                                                                            start_condition=final_open)

        kitchen_setup.motion_goals.hsrb_dishwasher_door_around(handle_name=handle_name,
                                                               tip_gripper_axis=tip_grasp_axis_push,
                                                               root_link=root_link,
                                                               tip_link=tip_link,
                                                               goal_angle=goal_angle_half,
                                                               start_condition=final_open,
                                                               end_condition=around_local_min)

        align_push_door_local_min = kitchen_setup.monitors.add_local_minimum_reached(name='align push door local min',
                                                                                     start_condition=around_local_min)

        kitchen_setup.motion_goals.add_align_to_push_door(root_link=root_link,
                                                          tip_link=tip_link,
                                                          door_handle=handle_name,
                                                          door_object=door_hinge_frame_id,
                                                          tip_gripper_axis=tip_grasp_axis_push,
                                                          weight=WEIGHT_ABOVE_CA,
                                                          goal_angle=goal_angle_half,
                                                          intermediate_point_scale=0.95,
                                                          start_condition=around_local_min,
                                                          end_condition=align_push_door_local_min)

        final_close = kitchen_setup.monitors.add_close_hsr_gripper(name='final close',
                                                                   start_condition=align_push_door_local_min)

        pre_push_local_min = kitchen_setup.monitors.add_local_minimum_reached(name='pre push local min',
                                                                              start_condition=final_close)

        kitchen_setup.motion_goals.add_pre_push_door(root_link=root_link,
                                                     tip_link=tip_link,
                                                     door_handle=handle_name,
                                                     weight=WEIGHT_ABOVE_CA,
                                                     door_object=door_hinge_frame_id,
                                                     start_condition=final_close,
                                                     end_condition=pre_push_local_min)

        full_open_joint = kitchen_setup.monitors.add_joint_position(name='full open joint',
                                                                    goal_state={hinge_joint: goal_angle_full},
                                                                    threshold=0.02,
                                                                    start_condition=pre_push_local_min)

        kitchen_setup.motion_goals.add_open_container(name='full open',
                                                      tip_link=tip_link,
                                                      environment_link=handle_name,
                                                      goal_joint_state=goal_angle_full,
                                                      start_condition=pre_push_local_min,
                                                      end_condition=full_open_joint)

        park_local_min = kitchen_setup.monitors.add_local_minimum_reached(name='park local min',
                                                                          start_condition=full_open_joint)

        kitchen_setup.motion_goals.add_take_pose(pose_keyword='park', start_condition=full_open_joint,
                                                 end_condition=park_local_min)

        kitchen_setup.monitors.add_end_motion(start_condition=park_local_min)

        kitchen_setup.motion_goals.allow_collision(env_name, gripper_group)
        kitchen_setup.execute(add_local_minimum_reached=False)


class TestCollisionAvoidanceGoals:

    def test_self_collision_avoidance_empty(self, zero_pose: HSRTestWrapper):
        zero_pose.allow_all_collisions()
        zero_pose.execute(expected_error_type=EmptyProblemException)
        current_state = god_map.world.state.to_position_dict()
        current_state = {k.short_name: v for k, v in current_state.items()}
        zero_pose.compare_joint_state(current_state, zero_pose.default_pose)

    def test_self_collision_avoidance(self, zero_pose: HSRTestWrapper):
        r_goal = PoseStamped()
        r_goal.header.frame_id = zero_pose.tip
        r_goal.pose.position.z = 0.5
        r_goal.pose.orientation.w = 1
        zero_pose.set_cart_goal(goal_pose=r_goal, tip_link=zero_pose.tip, root_link='map')
        zero_pose.execute()

    def test_self_collision_avoidance2(self, zero_pose: HSRTestWrapper):
        js = {
            'arm_flex_joint': -0.03,
            'arm_lift_joint': 0.0,
            'arm_roll_joint': -1.52,
            'head_pan_joint': -0.09,
            'head_tilt_joint': -0.62,
            'wrist_flex_joint': -1.55,
            'wrist_roll_joint': 0.11,
        }
        zero_pose.set_seed_configuration(js)
        zero_pose.allow_all_collisions()
        zero_pose.execute()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'hand_palm_link'
        goal_pose.pose.position.x = 0.5
        goal_pose.pose.orientation.w = 1
        zero_pose.set_cart_goal(goal_pose=goal_pose, tip_link=zero_pose.tip, root_link='map')
        zero_pose.execute()

    def test_attached_collision1(self, box_setup: HSRTestWrapper):
        box_name = 'asdf'
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(0.85, 0.3, .66)
        box_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        box_setup.add_box_to_world(box_name, (0.07, 0.04, 0.1), box_pose)
        box_setup.open_gripper()

        grasp_pose = deepcopy(box_pose)
        # grasp_pose.pose.position.x -= 0.05
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
                                                                          [0, -1, 0, 0],
                                                                          [1, 0, 0, 0],
                                                                          [0, 0, 0, 1]]))
        box_setup.set_cart_goal(goal_pose=grasp_pose, tip_link=box_setup.tip, root_link='map')
        box_setup.execute()
        box_setup.update_parent_link_of_group(box_name, box_setup.tip)

        base_goal = PoseStamped()
        base_goal.header.frame_id = box_setup.default_root
        base_goal.pose.position.x -= 0.5
        base_goal.pose.orientation.w = 1
        box_setup.move_base(base_goal)

        box_setup.close_gripper()

    def test_collision_avoidance(self, zero_pose: HSRTestWrapper):
        js = {'arm_flex_joint': -np.pi / 2}
        zero_pose.set_joint_goal(js)
        zero_pose.execute()

        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = 0.9
        p.pose.position.y = 0
        p.pose.position.z = 0.5
        p.pose.orientation.w = 1
        zero_pose.add_box_to_world(name='box', size=(1, 1, 0.01), pose=p)

        js = {'arm_flex_joint': 0}
        zero_pose.set_joint_goal(js, add_monitor=False)
        zero_pose.execute()

    #
    # def test_avoid_collision_touch_hard_threshold(self, box_setup: HSRTestWrapper):
    #     base_goal = PoseStamped()
    #     base_goal.header.frame_id = box_setup.default_root
    #     base_goal.pose.position.x = 0.2
    #     base_goal.pose.orientation.z = 1
    #     box_setup.teleport_base(base_goal)
    #
    #     box_setup.avoid_collision(min_distance=0.05, group1=box_setup.robot_name)
    #     box_setup.allow_self_collision()
    #
    #     base_goal = PoseStamped()
    #     base_goal.header.frame_id = 'base_footprint'
    #     base_goal.pose.position.x = -0.3
    #     base_goal.pose.orientation.w = 1
    #     box_setup.set_cart_goal(base_goal, tip_link='base_footprint', root_link='map', weight=WEIGHT_ABOVE_CA)
    #     box_setup.set_max_traj_length(30)
    #     box_setup.execute(add_local_minimum_reached=False)
    #     box_setup.check_cpi_geq(['base_link'], 0.048)
    #     box_setup.check_cpi_leq(['base_link'], 0.07)


class TestAddObject:
    def test_add(self, zero_pose):
        box1_name = 'box1'
        pose = PoseStamped()
        pose.header.frame_id = zero_pose.default_root
        pose.pose.orientation.w = 1
        pose.pose.position.x = 1
        zero_pose.add_box_to_world(name=box1_name,
                                   size=(1, 1, 1),
                                   pose=pose,
                                   parent_link='hand_palm_link')

        zero_pose.set_joint_goal({'arm_flex_joint': -0.7})
        zero_pose.execute()


class TestSUTURO:

    # TODO: add compare pose?
    def test_continuous_pointing(self, zero_pose):
        pub = rospy.Publisher('/human_pose', PointStamped, queue_size=10)

        zero_pose.continuous_pointing_head()
        zero_pose.execute(wait=False, add_local_minimum_reached=False)

        rospy.sleep(1)

        poses = []

        pose = PointStamped()
        pose.header.frame_id = 'map'
        pose.point.x = 1
        pose.point.z = 1

        poses.append(pose)

        pose2 = deepcopy(pose)
        pose2.point.y = 5

        poses.append(pose2)

        pose3 = deepcopy(pose2)

        pose3.point.x = 0.5
        pose3.point.y = 0

        poses.append(pose3)

        for pose in poses:
            pub.publish(pose)
            rospy.sleep(2)

        zero_pose.take_pose('park')
        zero_pose.execute()

    def test_open_door(self, door_setup: HSRTestWrapper):

        handle_name = "suturo_door/suturo_door_area:door_handle_inside"

        door_setup.open_gripper()

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = door_setup.tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.z = -1
        door_setup.set_align_planes_goal(tip_link=door_setup.tip,
                                         tip_normal=x_gripper,
                                         goal_normal=x_goal,
                                         root_link='map')

        door_setup.motion_goals.hsrb_door_handle_grasp(handle_name=handle_name, handle_bar_length=0.05)

        door_setup.execute()

        door_setup.close_gripper()

        door_setup.motion_goals.hsrb_open_door_goal(door_handle_link=handle_name, handle_limit=0.35,
                                                    hinge_limit=-0.8)

        door_setup.allow_all_collisions()

        door_setup.execute(add_local_minimum_reached=False)

        door_setup.open_gripper()

    def test_open_door_force_torque(self, door_setup: HSRTestWrapper):

        handle_name = "suturo_door/suturo_door_area:door_handle_inside"

        door_setup.open_gripper()

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = door_setup.tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.z = -1
        door_setup.motion_goals.add_align_planes(tip_link=door_setup.tip,
                                                 tip_normal=x_gripper,
                                                 goal_normal=x_goal,
                                                 root_link='map')

        offset = Vector3Stamped()
        offset.header.frame_id = 'hand_gripper_tool_frame'
        offset.vector.x = -0.1

        slep = door_setup.monitors.add_sleep(1000)
        force = door_setup.monitors.add_force_torque(ForceTorqueThresholds.DOOR.value, '')
        door_setup.motion_goals.hsrb_door_handle_grasp(handle_name=handle_name, handle_bar_length=0.05,
                                                       grasp_axis_offset=offset, end_condition=force)

        goal_point = PointStamped()
        goal_point.header.frame_id = 'base_footprint'
        goal_point.point.x = -0.1

        door_setup.motion_goals.add_cartesian_position_straight(root_link='map', tip_link='base_link',
                                                                goal_point=goal_point, start_condition=force)
        grasped = door_setup.monitors.add_local_minimum_reached(name='grasped monitor', start_condition=force)

        door_setup.monitors.add_end_motion(start_condition=grasped)
        door_setup.monitors.add_cancel_motion(f'not {force} and {slep} ',
                                              ObjectForceTorqueThresholdException('Door not touched!'))

        door_setup.allow_all_collisions()
        door_setup.execute(add_local_minimum_reached=False)

        door_setup.close_gripper()

        door_setup.motion_goals.hsrb_open_door_goal(door_handle_link=handle_name, handle_limit=0.35,
                                                    hinge_limit=-0.8)

        door_setup.allow_all_collisions()

        door_setup.execute(add_local_minimum_reached=False)

        door_setup.open_gripper()

    def test_open_hohc_simon(self, hohc_setup: HSRTestWrapper):
        hohc_opened_door_joint = 'suturo_shelf_hohc/shelf_hohc:shelf_door_right:joint'
        hohc_setup.set_env_state({hohc_opened_door_joint: 1.7})
        hohc_setup.open_gripper()

        left_handle = 'shelf_hohc:shelf_door_left:handle'
        left_door = 'shelf_hohc:shelf_door_left'

        hohc_setup.pre_pose_shelf_open(left_handle=left_handle,
                                       left_door=left_door,
                                       offset_x=0.03,
                                       offset_y=0.01,
                                       offset_z=-0.15)

        hohc_setup.execute()
        hohc_setup.close_gripper()

        hohc_setup.open_shelf_door(left_handle=left_handle, left_door=left_door)
        hohc_setup.execute()

    def test_reaching1(self, zero_pose: HSRTestWrapper):

        grasp_pose_1 = PoseStamped()
        grasp_pose_1.header.frame_id = 'map'
        grasp_pose_1.pose.position.x = 0.99999961185233
        grasp_pose_1.pose.position.y = 0.0001316214338790437
        grasp_pose_1.pose.position.z = 0.6900203701423123
        grasp_pose_1.pose.orientation.x = 0.5797565621267795
        grasp_pose_1.pose.orientation.y = 6.533426136710821e-05
        grasp_pose_1.pose.orientation.z = 0.8147895961626185
        grasp_pose_1.pose.orientation.w = -5.619679601192823e-05

        grasp_pose_2 = PoseStamped()
        grasp_pose_2.header.frame_id = 'map'
        grasp_pose_2.pose.position.x = 0.9999999674340407
        grasp_pose_2.pose.position.y = 1.1264868679568747e-05
        grasp_pose_2.pose.position.z = 0.7497550904127396
        grasp_pose_2.pose.orientation.x = -0.9999999992475002
        grasp_pose_2.pose.orientation.y = 2.203190918889499e-09
        grasp_pose_2.pose.orientation.z = -3.879432524505631e-05
        grasp_pose_2.pose.orientation.w = 2.979569784671264e-09

        grasp_pose_3 = PoseStamped()
        grasp_pose_3.header.frame_id = 'map'
        grasp_pose_3.pose.position.x = 1.0000098440969631
        grasp_pose_3.pose.position.y = -5.5826046789126e-07
        grasp_pose_3.pose.position.z = 0.7000118356590226
        grasp_pose_3.pose.orientation.x = -0.5000840140763921
        grasp_pose_3.pose.orientation.y = 0.49988866812121957
        grasp_pose_3.pose.orientation.z = -0.5001139088438252
        grasp_pose_3.pose.orientation.w = -0.4999133690252605

        grasp_pose_4 = PoseStamped()
        grasp_pose_4.header.frame_id = 'map'
        grasp_pose_4.pose.position.x = 0.9999945694917095
        grasp_pose_4.pose.position.y = 4.234772015936794e-05
        grasp_pose_4.pose.position.z = 0.7000060983590498
        grasp_pose_4.pose.orientation.x = -0.5000479580669888
        grasp_pose_4.pose.orientation.y = -0.4999377526131163
        grasp_pose_4.pose.orientation.z = -0.5000637350884022
        grasp_pose_4.pose.orientation.w = 0.49995054154847934

        grasp_pose_5 = PoseStamped()
        grasp_pose_5.header.frame_id = 'map'
        grasp_pose_5.pose.position.x = 0.9999945694917095
        grasp_pose_5.pose.position.y = 4.234772015936794e-05
        grasp_pose_5.pose.position.z = 0.6731324627618642
        grasp_pose_5.pose.orientation.x = -0.0020323559934585884
        grasp_pose_5.pose.orientation.y = 4.2785611489208335e-11
        grasp_pose_5.pose.orientation.z = -0.9999979347624254
        grasp_pose_5.pose.orientation.w = -7.953924864631972e-08

        grasp_states = {
            GraspTypes.FRONT: grasp_pose_1,
            GraspTypes.ABOVE: grasp_pose_2,
            GraspTypes.LEFT: grasp_pose_3,
            GraspTypes.RIGHT: grasp_pose_4,
            GraspTypes.BELOW: grasp_pose_5
        }

        box_name = 'asdf'
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(1, 0, 0.7)
        box_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        zero_pose.add_box_to_world(box_name, (0.07, 0.04, 0.1), box_pose)

        zero_pose.take_pose("pre_align_height")
        zero_pose.execute()

        zero_pose.open_gripper()

        for grasp in GraspTypes:
            zero_pose.motion_goals.add_motion_goal(motion_goal_class=Reaching.__name__,
                                                   object_name=box_name,
                                                   object_shape='box',
                                                   grasp=grasp.value,
                                                   align='test',
                                                   root_link='map',
                                                   tip_link='hand_gripper_tool_frame')

            zero_pose.allow_all_collisions()
            zero_pose.execute()

            root_link = god_map.world.search_for_link_name('map')
            tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
            m_P_g = (god_map.world.compute_fk(root_link, tip_link))
            # decimal needs to be one, because otherwise the test values need to be changed every time there's a giskard change
            compare_poses(m_P_g, grasp_states[grasp].pose, decimal=1)

            zero_pose.reset_base()
            zero_pose.take_pose("pre_align_height")
            zero_pose.execute()

        zero_pose.close_gripper()

    # TODO: Rework Test for actual Tray
    def test_funi_tray(self, zero_pose: HSRTestWrapper):
        # add actual Tray object (Object should be put in urdfs/meshes(?))
        box_name = 'Tray'
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(1, 0, 0.7)
        box_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        zero_pose.add_box_to_world(box_name, (0.345, 0.455, 0.02), box_pose)

        zero_pose.take_pose("pre_align_height")
        zero_pose.execute()

        zero_pose.open_gripper()

        zero_pose.motion_goals.add_joint_position(goal_state={'wrist_roll_joint': -(0.5 * pi)}, start_condition='',
                                                  hold_condition='', end_condition='')

        zero_pose.execute()

        zero_pose.motion_goals.add_motion_goal(motion_goal_class=Reaching.__name__,
                                               object_name=box_name,
                                               object_shape='box',
                                               grasp=GraspTypes.FRONT.value,
                                               align='test',
                                               root_link='map',
                                               tip_link='hand_gripper_tool_frame')

        zero_pose.allow_all_collisions()
        zero_pose.execute()

        zero_pose.close_gripper()

    def test_placing_motion(self, zero_pose: HSRTestWrapper):
        box_name = 'asdf'
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(1, 0, 0.7)
        box_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        zero_pose.add_box_to_world(box_name, (0.07, 0.04, 0.1), box_pose)

        zero_pose.take_pose("pre_align_height")
        zero_pose.execute()

        zero_pose.motion_goals.add_motion_goal(motion_goal_class=Placing.__name__,
                                               goal_pose=box_pose,
                                               align='',
                                               grasp=GraspTypes.ABOVE.value,
                                               root_link='map',
                                               tip_link='hand_gripper_tool_frame')

        zero_pose.allow_all_collisions()
        zero_pose.execute()

        zero_pose.open_gripper()

        zero_pose.motion_goals.add_motion_goal(motion_goal_class=TakePose.__name__,
                                               pose_keyword='park')

        zero_pose.close_gripper()

    def test_vertical_motion_up(self, zero_pose: HSRTestWrapper):

        vertical_motion_pose = Pose()
        vertical_motion_pose.position.x = 0.17102731790942596
        vertical_motion_pose.position.y = -0.13231521471220506
        vertical_motion_pose.position.z = 0.7119274770524749
        vertical_motion_pose.orientation.x = 0.5067617681482114
        vertical_motion_pose.orientation.y = -0.45782201564184877
        vertical_motion_pose.orientation.z = 0.5271017946406412
        vertical_motion_pose.orientation.w = 0.5057224638312487

        zero_pose.motion_goals.add_motion_goal(motion_goal_class=TakePose.__name__,
                                               pose_keyword='park')

        zero_pose.allow_self_collision()
        zero_pose.execute()

        sleep = zero_pose.monitors.add_sleep(seconds=0.1)
        local_min = zero_pose.monitors.add_local_minimum_reached()

        action = 'grasping'
        zero_pose.motion_goals.add_motion_goal(motion_goal_class=VerticalMotion.__name__,
                                               action=action,
                                               distance=0.02,
                                               root_link='base_footprint',
                                               tip_link='hand_palm_link',
                                               start_condition=sleep,
                                               end_condition=local_min)

        zero_pose.monitors.add_end_motion(start_condition=f'{sleep} and {local_min}')

        zero_pose.allow_self_collision()
        zero_pose.execute(add_local_minimum_reached=False)

        root_link = god_map.world.search_for_link_name('map')
        tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
        m_P_g = (god_map.world.compute_fk(root_link, tip_link))

        compare_poses(m_P_g, vertical_motion_pose)

    def test_retracting_hand(self, zero_pose: HSRTestWrapper):

        retracting_hand_pose = Pose()
        retracting_hand_pose.position.x = 0.14963260254170513
        retracting_hand_pose.position.y = 0.16613649117825122
        retracting_hand_pose.position.z = 0.6717532654948288
        retracting_hand_pose.orientation.x = 0.5066648708788183
        retracting_hand_pose.orientation.y = -0.45792002831875167
        retracting_hand_pose.orientation.z = 0.5270228996549048
        retracting_hand_pose.orientation.w = 0.5058130282241059

        zero_pose.motion_goals.add_motion_goal(motion_goal_class='TakePose',
                                               pose_keyword='park')

        zero_pose.allow_self_collision()
        zero_pose.execute()

        sleep = zero_pose.monitors.add_sleep(seconds=0.1)
        local_min = zero_pose.monitors.add_local_minimum_reached()

        zero_pose.motion_goals.add_motion_goal(motion_goal_class='Retracting',
                                               distance=0.3,
                                               reference_frame='hand_palm_link',
                                               root_link='map',
                                               tip_link='hand_palm_link',
                                               start_condition=sleep)

        zero_pose.monitors.add_end_motion(start_condition=f'{local_min} and {sleep}')

        zero_pose.allow_self_collision()
        zero_pose.execute(add_local_minimum_reached=False)

        root_link = god_map.world.search_for_link_name('map')
        tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
        m_P_g = (god_map.world.compute_fk(root_link, tip_link))

        compare_poses(m_P_g, retracting_hand_pose)

    def test_retracting_base(self, zero_pose: HSRTestWrapper):

        retraction_base_pose = PoseStamped()
        retraction_base_pose.header.frame_id = 'map'
        retraction_base_pose.pose.position.x = -0.12533144864637413
        retraction_base_pose.pose.position.y = 0.07795010184370622
        retraction_base_pose.pose.position.z = 0.894730930853242
        retraction_base_pose.pose.orientation.x = 0.014859073808224462
        retraction_base_pose.pose.orientation.y = -0.00015418547016511882
        retraction_base_pose.pose.orientation.z = 0.9998893945231346
        retraction_base_pose.pose.orientation.w = -0.0006187669689175172

        sleep = zero_pose.monitors.add_sleep(seconds=0.1)
        local_min = zero_pose.monitors.add_local_minimum_reached()

        zero_pose.motion_goals.add_motion_goal(motion_goal_class='Retracting',
                                               distance=0.3,
                                               reference_frame='base_footprint',
                                               root_link='map',
                                               tip_link='hand_palm_link',
                                               start_condition=sleep)

        zero_pose.monitors.add_end_motion(start_condition=f'{local_min} and {sleep}')

        zero_pose.allow_self_collision()
        zero_pose.execute(add_local_minimum_reached=False)

        root_link = god_map.world.search_for_link_name('map')
        tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
        m_P_g = (god_map.world.compute_fk(root_link, tip_link))

        compare_poses(m_P_g, retraction_base_pose.pose)

    def test_align_height(self, zero_pose: HSRTestWrapper):
        execute_from_above = [False, True]

        align_pose1 = PoseStamped()
        align_pose1.header.frame_id = 'map'
        align_pose1.pose.position.x = 0.3670559556308583
        align_pose1.pose.position.y = 0.00022361096354857893
        align_pose1.pose.position.z = 0.7728331262049145
        align_pose1.pose.orientation.x = 0.6930355696535618
        align_pose1.pose.orientation.y = 0.0002441417024468236
        align_pose1.pose.orientation.z = 0.720903306535367
        align_pose1.pose.orientation.w = -0.0002494316878550612

        align_pose2 = PoseStamped()
        align_pose2.header.frame_id = 'map'
        align_pose2.pose.position.x = 0.2943309402390854
        align_pose2.pose.position.y = -0.0004960369085802845
        align_pose2.pose.position.z = 0.7499314955573722
        align_pose2.pose.orientation.x = 0.999999932400925
        align_pose2.pose.orientation.y = 0.0003514656228904682
        align_pose2.pose.orientation.z = -0.00010802805208618605
        align_pose2.pose.orientation.w = 3.7968463309867553e-08

        align_states = {
            False: align_pose1,
            True: align_pose2,
        }

        for mode in execute_from_above:
            zero_pose.motion_goals.add_motion_goal(motion_goal_class='TakePose',
                                                   pose_keyword='pre_align_height')

            zero_pose.allow_self_collision()
            zero_pose.execute()

            action = 'grasping'
            from_above = mode

            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = 1
            target_pose.pose.position.z = 0.7

            zero_pose.motion_goals.add_motion_goal(motion_goal_class=AlignHeight.__name__,
                                                   from_above=from_above,
                                                   object_name='',
                                                   goal_pose=target_pose,
                                                   object_height=0.1,
                                                   root_link='map',
                                                   tip_link='hand_palm_link')

            zero_pose.allow_self_collision()
            zero_pose.execute()

            root_link = god_map.world.search_for_link_name('map')
            tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
            cord_data = (god_map.world.compute_fk(root_link, tip_link))

            compare_poses(cord_data, align_states[mode].pose)

    # Maybe change compare poses to fingertips and not tool_frame
    def test_tilting(self, zero_pose: HSRTestWrapper):
        directions = ['left', 'right']

        # Orientation for tilt_pose 1 needs to be negative despite given parameters being returned as positives...
        tilt_pose1 = PoseStamped()
        tilt_pose1.header.frame_id = 'map'
        tilt_pose1.pose.position.x = 0.3862282703183651
        tilt_pose1.pose.position.y = 0.07997985276116013
        tilt_pose1.pose.position.z = 0.695562902503049
        tilt_pose1.pose.orientation.x = 0.02036729579358757
        tilt_pose1.pose.orientation.y = -0.09918407993790013
        tilt_pose1.pose.orientation.z = 0.7016143119255045
        tilt_pose1.pose.orientation.w = 0.7053262003145989

        tilt_pose2 = PoseStamped()
        tilt_pose2.header.frame_id = 'map'
        tilt_pose2.pose.position.x = 0.4011968051112429
        tilt_pose2.pose.position.y = 0.07997985276116013
        tilt_pose2.pose.position.z = 0.6997425428565389
        tilt_pose2.pose.orientation.x = -0.7013959300921285
        tilt_pose2.pose.orientation.y = 0.7062105656448003
        tilt_pose2.pose.orientation.z = -0.02684219924309636
        tilt_pose2.pose.orientation.w = -0.09268161933006579

        tilt_states = {
            'left': tilt_pose1,
            'right': tilt_pose2,
        }

        zero_pose.motion_goals.add_motion_goal(motion_goal_class='TakePose',
                                               pose_keyword='pre_align_height')

        zero_pose.allow_self_collision()
        zero_pose.execute()

        for direction in directions:
            sleep = zero_pose.monitors.add_sleep(seconds=0.1)
            local_min = zero_pose.monitors.add_local_minimum_reached()

            zero_pose.motion_goals.add_motion_goal(motion_goal_class='Tilting',
                                                   direction=direction,
                                                   angle=1.4,
                                                   start_condition='',
                                                   end_condition=local_min)

            zero_pose.monitors.add_end_motion(start_condition=f'{sleep} and {local_min}')

            zero_pose.allow_self_collision()
            zero_pose.execute(add_local_minimum_reached=False)

            cord_data = (zero_pose.compute_fk_pose('map', 'hand_l_finger_tip_frame'))

            compare_poses(cord_data.pose, tilt_states[direction].pose)

    def test_take_pose(self, zero_pose: HSRTestWrapper):
        poses = ['park', 'perceive', 'assistance', 'pre_align_height', 'carry']

        park_pose = PoseStamped()
        park_pose.header.frame_id = 'map'
        park_pose.pose.position.x = 0.1710261260244742
        park_pose.pose.position.y = -0.13231889092341187
        park_pose.pose.position.z = 0.6919283778314267
        park_pose.pose.orientation.x = 0.5067619888164565
        park_pose.pose.orientation.y = -0.45782179605285284
        park_pose.pose.orientation.z = 0.5271015813648557
        park_pose.pose.orientation.w = 0.5057226637915272

        perceive_pose = PoseStamped()
        perceive_pose.header.frame_id = 'map'
        perceive_pose.pose.position.x = 0.1710444625574895
        perceive_pose.pose.position.y = 0.2883150465871069
        perceive_pose.pose.position.z = 0.9371745637108605
        perceive_pose.pose.orientation.x = -0.5063851509844108
        perceive_pose.pose.orientation.y = -0.457448402898974
        perceive_pose.pose.orientation.z = -0.527458211023338
        perceive_pose.pose.orientation.w = 0.5060660758949697

        assistance_pose = PoseStamped()
        assistance_pose.header.frame_id = 'map'
        assistance_pose.pose.position.x = 0.18333071333185327
        assistance_pose.pose.position.y = -0.1306120975368269
        assistance_pose.pose.position.z = 0.7050680498627263
        assistance_pose.pose.orientation.x = 0.024667116882362873
        assistance_pose.pose.orientation.y = -0.6819662708507778
        assistance_pose.pose.orientation.z = 0.7305124281436971
        assistance_pose.pose.orientation.w = -0.025790135598626814

        pre_align_height_pose = PoseStamped()
        pre_align_height_pose.header.frame_id = 'map'
        pre_align_height_pose.pose.position.x = 0.36718508844870135
        pre_align_height_pose.pose.position.y = 0.07818733568602311
        pre_align_height_pose.pose.position.z = 0.6872325515876044
        pre_align_height_pose.pose.orientation.x = 0.6925625964573222
        pre_align_height_pose.pose.orientation.y = 0.0008342119786388634
        pre_align_height_pose.pose.orientation.z = 0.7213572801204168
        pre_align_height_pose.pose.orientation.w = 0.0001688074098573283

        carry_pose = PoseStamped()
        carry_pose.header.frame_id = 'map'
        carry_pose.pose.position.x = 0.4997932992635221
        carry_pose.pose.position.y = 0.06601541592028287
        carry_pose.pose.position.z = 0.6519470331487148
        carry_pose.pose.orientation.x = 0.49422863353080027
        carry_pose.pose.orientation.y = 0.5199402328561551
        carry_pose.pose.orientation.z = 0.4800020391690775
        carry_pose.pose.orientation.w = 0.5049735185624021

        assert_poses = {
            'park': park_pose.pose,
            'perceive': perceive_pose.pose,
            'assistance': assistance_pose.pose,
            'pre_align_height': pre_align_height_pose.pose,
            'carry': carry_pose.pose
        }

        for pose in poses:
            zero_pose.motion_goals.add_motion_goal(motion_goal_class=TakePose.__name__,
                                                   pose_keyword=pose,
                                                   max_velocity=None)

            zero_pose.allow_self_collision()
            zero_pose.execute()

            root_link = god_map.world.search_for_link_name('map')
            tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
            m_P_g = (god_map.world.compute_fk(root_link, tip_link))

            compare_poses(m_P_g, assert_poses[pose])

    # # TODO: If ever relevant for SuTuRo, add proper Test behaviour
    # def test_mixing(self, zero_pose: HSRTestWrapper):
    #     # FIXME: Cant use traj_time_in_seconds in standalone mode
    #     zero_pose.motion_goals.add_motion_goal(motion_goal_class='Mixing',
    #                                            mixing_time=20)
    #
    #     zero_pose.allow_self_collision()
    #     zero_pose.plan_and_execute()
    #
    # def test_joint_rotation_goal_continuous(self, zero_pose: HSRTestWrapper):
    #     # FIXME: Use compare_pose similar to other tests
    #     # FIXME: Cant use traj_time_in_seconds in standalone mode
    #     zero_pose.motion_goals.add_motion_goal(motion_goal_class='JointRotationGoalContinuous',
    #                                            joint_name='arm_roll_joint',
    #                                            joint_center=0.0,
    #                                            joint_range=0.2,
    #                                            trajectory_length=20,
    #                                            target_speed=1,
    #                                            period_length=1.0)
    #
    #     zero_pose.allow_self_collision()
    #     zero_pose.plan_and_execute()

    def test_keep_rotation_goal(self, zero_pose: HSRTestWrapper):

        keep_rotation_pose = PoseStamped()
        keep_rotation_pose.header.frame_id = 'map'
        keep_rotation_pose.pose.position.x = 0.9402845292991675
        keep_rotation_pose.pose.position.y = -0.7279803708852316
        keep_rotation_pose.pose.position.z = 0.8754144094343888  # 0.8994121023446626
        keep_rotation_pose.pose.orientation.x = 0.015000397751939919
        keep_rotation_pose.pose.orientation.y = -2.1716350146486636e-07
        keep_rotation_pose.pose.orientation.z = 0.999887487627967
        keep_rotation_pose.pose.orientation.w = 1.2339723016403797e-05

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position.x = 1
        base_goal.pose.position.y = -1
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi / 2, [0, 0, 1]))
        zero_pose.set_cart_goal(base_goal, root_link=god_map.world.root_link_name, tip_link='base_footprint')

        zero_pose.motion_goals.add_motion_goal(motion_goal_class='KeepRotationGoal',
                                               tip_link='hand_palm_link')

        zero_pose.allow_self_collision()
        zero_pose.execute()

        root_link = god_map.world.search_for_link_name('map')
        tip_link = god_map.world.search_for_link_name('hand_gripper_tool_frame')
        m_P_g = (god_map.world.compute_fk(root_link, tip_link))

        compare_poses(m_P_g, keep_rotation_pose.pose)

    # def test_hsr_open_close_gripper(self, zero_pose: HSRTestWrapper):
    #     if 'GITHUB_WORKFLOW' in os.environ:
    #         return True
    #     echo = rospy.Publisher('/hsrb/gripper_controller/grasp/result', GripperApplyEffortActionResult,
    #                            queue_size=1)
    #     gripper_open = zero_pose.monitors.add_open_hsr_gripper(name='open')
    #     gripper_closed = zero_pose.monitors.add_close_hsr_gripper(name='close',
    #                                                               start_condition=gripper_open)
    #     zero_pose.monitors.add_end_motion(start_condition=gripper_closed)
    #     zero_pose.execute(add_local_minimum_reached=False, wait=False)
    #
    #     for i in range(2):
    #         msg: GripperApplyEffortActionGoal = rospy.wait_for_message('/hsrb/gripper_controller/grasp/goal',
    #                                                                    GripperApplyEffortActionGoal)fh
    #         result = GripperApplyEffortActionResult()
    #         result.status.goal_id = msg.goal_id
    #         echo.publish(result)
    #     result = zero_pose.get_result()
    #     assert result.error.code == GiskardError.SUCCESS


class TestKitchen:
    def test_kitchen_island_open(self, kitchen_setup: HSRTestWrapper):
        open_drawers = []

        # Left Drawers
        handle_frame_id_lower = 'iai_kitchen/kitchen_island_left_lower_drawer_handle'
        handle_name_lower = 'kitchen_island_left_lower_drawer_handle'
        drawer_fix_lower = {'kitchen_island_left_lower_drawer_main_joint': 0}
        handle_frame_id_upper = 'iai_kitchen/kitchen_island_left_upper_drawer_handle'
        handle_name_upper = 'kitchen_island_left_upper_drawer_handle'
        drawer_fix_upper = {'kitchen_island_left_upper_drawer_main_joint': 0}

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.2, 1, 0)
        base_goal.pose.orientation.x = 0
        base_goal.pose.orientation.y = 0
        base_goal.pose.orientation.z = 1
        base_goal.pose.orientation.w = 0.00000013

        open_drawers.append((deepcopy(base_goal), handle_frame_id_lower, handle_name_lower, drawer_fix_lower))
        open_drawers.append((deepcopy(base_goal), handle_frame_id_upper, handle_name_upper, drawer_fix_upper))

        # Middle Drawers
        handle_frame_id_lower = 'iai_kitchen/kitchen_island_middle_lower_drawer_handle'
        handle_name_lower = 'kitchen_island_middle_lower_drawer_handle'
        drawer_fix_lower = {'kitchen_island_middle_lower_drawer_main_joint': 0}
        handle_frame_id_upper = 'iai_kitchen/kitchen_island_middle_upper_drawer_handle'
        handle_name_upper = 'kitchen_island_middle_upper_drawer_handle'
        drawer_fix_upper = {'kitchen_island_middle_upper_drawer_main_joint': 0}

        base_goal.pose.position = Point(0.2, 1.5, 0)

        open_drawers.append((deepcopy(base_goal), handle_frame_id_lower, handle_name_lower, drawer_fix_lower))
        open_drawers.append((deepcopy(base_goal), handle_frame_id_upper, handle_name_upper, drawer_fix_upper))

        # Right Drawers
        handle_frame_id_lower = 'iai_kitchen/kitchen_island_right_lower_drawer_handle'
        handle_name_lower = 'kitchen_island_right_lower_drawer_handle'
        drawer_fix_lower = {'kitchen_island_right_lower_drawer_main_joint': 0}
        handle_frame_id_upper = 'iai_kitchen/kitchen_island_right_upper_drawer_handle'
        handle_name_upper = 'kitchen_island_right_upper_drawer_handle'
        drawer_fix_upper = {'kitchen_island_right_upper_drawer_main_joint': 0}

        base_goal.pose.position = Point(0.2, 2.5, 0)

        open_drawers.append((deepcopy(base_goal), handle_frame_id_lower, handle_name_lower, drawer_fix_lower))
        open_drawers.append((deepcopy(base_goal), handle_frame_id_upper, handle_name_upper, drawer_fix_upper))

        for goal, handle_frame_id, handle_name, drawer_fix in open_drawers:
            self.open_sequence(goal, handle_name, handle_frame_id, drawer_fix, kitchen_setup)
            self.close_sequence(handle_name, handle_frame_id, kitchen_setup)

    def test_kitchen_sink_open(self, kitchen_setup: HSRTestWrapper):
        open_drawers = []

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.2, 1, 0)
        base_goal.pose.orientation.w = 1

        # Left Drawers
        handle_frame_id_bottom = 'iai_kitchen/sink_area_left_bottom_drawer_handle'
        handle_name_bottom = 'sink_area_left_bottom_drawer_handle'
        drawer_fix_dict = {'sink_area_left_bottom_drawer_main_joint': 0}
        open_drawers.append(
            (deepcopy(base_goal), handle_frame_id_bottom, handle_name_bottom, deepcopy(drawer_fix_dict)))

        handle_frame_id_middle = 'iai_kitchen/sink_area_left_middle_drawer_handle'
        handle_name_middle = 'sink_area_left_middle_drawer_handle'
        drawer_fix_dict = {'sink_area_left_middle_drawer_main_joint': 0}
        open_drawers.append(
            (deepcopy(base_goal), handle_frame_id_middle, handle_name_middle, deepcopy(drawer_fix_dict)))

        handle_frame_id_upper = 'iai_kitchen/sink_area_left_upper_drawer_handle'
        handle_name_upper = 'sink_area_left_upper_drawer_handle'
        drawer_fix_dict = {'sink_area_left_upper_drawer_main_joint': 0}
        open_drawers.append((deepcopy(base_goal), handle_frame_id_upper, handle_name_upper, deepcopy(drawer_fix_dict)))

        # Trash Drawer
        handle_frame_id_trash = 'iai_kitchen/sink_area_trash_drawer_handle'
        handle_name_trash = 'sink_area_trash_drawer_handle'
        drawer_fix_dict = {'sink_area_trash_drawer_main_joint': 0}

        base_goal.pose.position = Point(0.2, -0.2, 0)

        open_drawers.append((deepcopy(base_goal), handle_frame_id_trash, handle_name_trash, deepcopy(drawer_fix_dict)))

        for goal, handle_frame_id, handle_name, drawer_fix in open_drawers:
            self.open_sequence(goal, handle_name, handle_frame_id, drawer_fix, kitchen_setup)
            self.close_sequence(handle_name, handle_frame_id, kitchen_setup)

    def test_kitchen_oven_open(self, kitchen_setup: HSRTestWrapper):
        open_drawers = []

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0.2, 2, 0)
        base_goal.pose.orientation.w = 1

        handle_frame_id_bottom = 'iai_kitchen/oven_area_area_left_drawer_handle'
        handle_name_bottom = 'oven_area_area_left_drawer_handle'
        drawer_fix_dict = {'oven_area_area_left_drawer_main_joint': 0}
        open_drawers.append(
            (deepcopy(base_goal), handle_frame_id_bottom, handle_name_bottom, deepcopy(drawer_fix_dict)))

        handle_frame_id_bottom = 'iai_kitchen/oven_area_area_right_drawer_handle'
        handle_name_bottom = 'oven_area_area_right_drawer_handle'
        drawer_fix_dict = {'oven_area_area_right_drawer_main_joint': 0}
        open_drawers.append(
            (deepcopy(base_goal), handle_frame_id_bottom, handle_name_bottom, deepcopy(drawer_fix_dict)))

        handle_frame_id_lower = 'iai_kitchen/oven_area_area_middle_lower_drawer_handle'
        handle_name_lower = 'oven_area_area_middle_lower_drawer_handle'
        drawer_fix_lower = {'oven_area_area_middle_lower_drawer_main_joint': 0}
        handle_frame_id_upper = 'iai_kitchen/oven_area_area_middle_upper_drawer_handle'
        handle_name_upper = 'oven_area_area_middle_upper_drawer_handle'
        drawer_fix_upper = {'oven_area_area_middle_upper_drawer_main_joint': 0}

        open_drawers.append((deepcopy(base_goal), handle_frame_id_upper, handle_name_upper, drawer_fix_upper))
        open_drawers.append((deepcopy(base_goal), handle_frame_id_lower, handle_name_lower, drawer_fix_lower))

        for goal, handle_frame_id, handle_name, drawer_fix in open_drawers:
            self.open_sequence(goal, handle_name, handle_frame_id, drawer_fix, kitchen_setup)
            self.close_sequence(handle_name, handle_frame_id, kitchen_setup)

    def open_sequence(self,
                      goal: PoseStamped,
                      handle_name: str,
                      handle_frame_id: str,
                      drawer_fix: Dict[str, float],
                      kitchen_setup: HSRTestWrapper):

        kitchen_setup.open_gripper()
        kitchen_setup.take_pose('park')
        kitchen_setup.execute()
        kitchen_setup.allow_all_collisions()
        kitchen_setup.move_base(goal)
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = -1
        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id
        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.tip
        tip_grasp_axis.vector.x = 1
        # %% phase 1
        bar_grasped = kitchen_setup.monitors.add_distance_to_line(name='bar grasped',
                                                                  root_link=kitchen_setup.default_root,
                                                                  tip_link=kitchen_setup.tip,
                                                                  center_point=bar_center,
                                                                  line_axis=bar_axis,
                                                                  line_length=.4)
        kitchen_setup.motion_goals.add_grasp_bar(root_link=kitchen_setup.default_root,
                                                 tip_link=kitchen_setup.tip,
                                                 tip_grasp_axis=tip_grasp_axis,
                                                 bar_center=bar_center,
                                                 bar_axis=bar_axis,
                                                 bar_length=.4,
                                                 name='grasp bar',
                                                 end_condition=bar_grasped)
        kitchen_setup.motion_goals.add_joint_position(goal_state=drawer_fix,
                                                      end_condition=bar_grasped)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.tip
        x_gripper.vector.z = 1
        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        kitchen_setup.motion_goals.add_align_planes(tip_link=kitchen_setup.tip,
                                                    tip_normal=x_gripper,
                                                    goal_normal=x_goal,
                                                    root_link='map',
                                                    name='orient to door',
                                                    end_condition=bar_grasped)
        # %% phase 2 open door
        door_open = kitchen_setup.monitors.add_local_minimum_reached(name='door open',
                                                                     start_condition=bar_grasped)
        kitchen_setup.motion_goals.add_open_container(tip_link=kitchen_setup.tip,
                                                      environment_link=handle_name,
                                                      goal_joint_state=1.5,
                                                      name='open door',
                                                      start_condition=bar_grasped,
                                                      end_condition=door_open)
        kitchen_setup.allow_self_collision()
        kitchen_setup.monitors.add_end_motion(start_condition=door_open)
        kitchen_setup.execute(add_local_minimum_reached=False)
        kitchen_setup.close_gripper()

    def close_sequence(self,
                       handle_name: str,
                       handle_frame_id: str,
                       kitchen_setup: HSRTestWrapper):

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = -1
        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id
        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.tip
        tip_grasp_axis.vector.x = 1
        # %% phase 1
        bar_grasped = kitchen_setup.monitors.add_distance_to_line(name='bar grasped',
                                                                  root_link=kitchen_setup.default_root,
                                                                  tip_link=kitchen_setup.tip,
                                                                  center_point=bar_center,
                                                                  line_axis=bar_axis,
                                                                  line_length=.4)
        kitchen_setup.motion_goals.add_grasp_bar(root_link=kitchen_setup.default_root,
                                                 tip_link=kitchen_setup.tip,
                                                 tip_grasp_axis=tip_grasp_axis,
                                                 bar_center=bar_center,
                                                 bar_axis=bar_axis,
                                                 bar_length=.4,
                                                 name='grasp bar',
                                                 end_condition=bar_grasped)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.tip
        x_gripper.vector.z = 1
        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        kitchen_setup.motion_goals.add_align_planes(tip_link=kitchen_setup.tip,
                                                    tip_normal=x_gripper,
                                                    goal_normal=x_goal,
                                                    root_link='map',
                                                    name='orient to door',
                                                    end_condition=bar_grasped)
        # %% phase 2 open door
        door_open = kitchen_setup.monitors.add_local_minimum_reached(name='door open',
                                                                     start_condition=bar_grasped)
        kitchen_setup.motion_goals.add_close_container(tip_link=kitchen_setup.tip,
                                                       environment_link=handle_name,
                                                       name='open door',
                                                       start_condition=bar_grasped,
                                                       end_condition=door_open)
        kitchen_setup.allow_all_collisions()
        kitchen_setup.monitors.add_end_motion(start_condition=door_open)
        kitchen_setup.execute(add_local_minimum_reached=False)
