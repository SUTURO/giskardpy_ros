#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3Stamped, Point

from giskardpy.data_types.suturo_types import ForceTorqueThresholds
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros1 import tfwrapper as tf

rospy.init_node('giskard_demo')

gis = GiskardWrapper()

handle_left_id = 'shelf_billy:shelf_billy:shelf_door_left:handle'
hinge_left_joint = 'shelf_billy:shelf_billy:shelf_door_left:joint'
handle_right_id = 'shelf_billy:shelf_billy:shelf_door_right:handle'
hinge_right_joint = 'shelf_billy:shelf_billy:shelf_door_right:joint'

handles = [(handle_left_id, hinge_left_joint), (handle_right_id, hinge_right_joint)]
second = False

for (handle_id, hinge_joint) in handles:
    base_pose = PoseStamped()
    base_pose.header.frame_id = 'map'
    base_pose.pose.position.x = 4.3
    base_pose.pose.position.y = 4
    base_pose.pose.orientation.w = 1

    odom = gis.monitors.add_local_minimum_reached()
    gis.monitors.add_open_hsr_gripper()
    joint_reset = gis.monitors.add_joint_position(goal_state={hinge_joint: 0})

    gis.motion_goals.add_joint_position(goal_state={hinge_joint: 0}, name='door hinge monitor')
    gis.motion_goals.add_cartesian_pose(root_link='map', tip_link='base_footprint', goal_pose=base_pose)

    gis.motion_goals.add_take_pose(pose_keyword='park')
    joints = gis.monitors.add_joint_position(goal_state={'head_pan_joint': 0.0,
                                                         'head_tilt_joint': 0.0,
                                                         'arm_lift_joint': 0.0,
                                                         'arm_flex_joint': 0.0,
                                                         'arm_roll_joint': -1.5,
                                                         'wrist_flex_joint': -1.5,
                                                         'wrist_roll_joint': 0.0},
                                             threshold=0.05,
                                             name='park arms monitor')

    gis.monitors.add_end_motion(start_condition=f'{joint_reset} and {odom} and {joints}')
    gis.execute()

    bar_center = PointStamped()
    bar_center.header.frame_id = handle_id

    bar_axis = Vector3Stamped()
    bar_axis.header.frame_id = handle_id
    bar_axis.vector.y = 1

    tip_grasp_axis = Vector3Stamped()
    tip_grasp_axis.header.frame_id = 'hand_gripper_tool_frame'
    tip_grasp_axis.vector.x = 1

    x_gripper = Vector3Stamped()
    x_gripper.header.frame_id = 'hand_gripper_tool_frame'
    x_gripper.vector.z = 1

    x_goal = Vector3Stamped()
    x_goal.header.frame_id = handle_id
    x_goal.vector.z = 1

    pre_grasp_offset = Vector3Stamped()
    pre_grasp_offset.header.frame_id = handle_id
    pre_grasp_offset.vector.z = -0.1

    ft_offset = Vector3Stamped()
    ft_offset.header.frame_id = handle_id
    ft_offset.vector.z = 0.1

    gis.monitors.add_open_hsr_gripper()

    gis.motion_goals.add_align_planes(name='pre grasp align',
                                      tip_link='hand_gripper_tool_frame',
                                      tip_normal=x_gripper,
                                      goal_normal=x_goal,
                                      root_link='map')

    local_min_pre_grasp = gis.monitors.add_local_minimum_reached(name='pre grasp local min')
    gis.motion_goals.add_grasp_bar_offset(name='pre grasp bar',
                                          root_link='map',
                                          tip_link='hand_gripper_tool_frame',
                                          tip_grasp_axis=tip_grasp_axis,
                                          bar_center=bar_center,
                                          bar_axis=bar_axis,
                                          bar_length=0.001,
                                          grasp_axis_offset=pre_grasp_offset,
                                          end_condition=local_min_pre_grasp)

    ft_monitor = gis.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.DOOR.value,
                                               start_condition=local_min_pre_grasp)
    gis.motion_goals.add_grasp_bar_offset(name='grasp bar',
                                          root_link='map',
                                          tip_link='hand_gripper_tool_frame',
                                          tip_grasp_axis=tip_grasp_axis,
                                          bar_center=bar_center,
                                          bar_axis=bar_axis,
                                          bar_length=0.001,
                                          grasp_axis_offset=ft_offset,
                                          start_condition=local_min_pre_grasp,
                                          end_condition=ft_monitor)

    goal_point = PointStamped()
    goal_point.header.frame_id = 'base_link'

    handle_retract_direction = Vector3Stamped()
    handle_retract_direction.header.frame_id = handle_id
    handle_retract_direction.vector.z = -0.1

    base_retract = tf.transform_vector(goal_point.header.frame_id, handle_retract_direction)

    goal_point.point = Point(base_retract.vector.x, base_retract.vector.y, base_retract.vector.z)

    gis.motion_goals.add_cartesian_position_straight(root_link='map', tip_link='base_link',
                                                     goal_point=goal_point, start_condition=ft_monitor)
    grasped = gis.monitors.add_cartesian_position(root_link='map', tip_link='base_link', goal_point=goal_point,
                                                  name='grasped monitor', start_condition=ft_monitor)

    close_gripper = gis.monitors.add_close_hsr_gripper(start_condition=grasped)

    gis.monitors.add_end_motion(start_condition=close_gripper)
    gis.execute()

    if second:
        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_id
        x_goal.vector.z = 1

        x_base = Vector3Stamped()
        x_base.header.frame_id = 'base_link'
        x_base.vector.y = -1

        gis.motion_goals.add_align_planes(goal_normal=x_goal, tip_link='base_link', tip_normal=x_base, root_link='map')
        gis.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame', environment_link=handle_id)
    else:
        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_id
        x_goal.vector.z = 1

        x_base = Vector3Stamped()
        x_base.header.frame_id = 'base_link'
        x_base.vector.y = 1

        gis.motion_goals.add_align_planes(goal_normal=x_goal, tip_link='base_link', tip_normal=x_base, root_link='map')
        gis.motion_goals.add_close_container(tip_link='hand_gripper_tool_frame', environment_link=handle_id)
        second = True

    local_min = gis.monitors.add_local_minimum_reached()
    open_gripper = gis.monitors.add_open_hsr_gripper(start_condition=local_min)

    gis.monitors.add_end_motion(start_condition=open_gripper)
    gis.execute()
