#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped, PointStamped, Vector3, PoseStamped, Point, PoseWithCovarianceStamped, \
    QuaternionStamped
from rospy import Publisher

import giskardpy_ros.ros1.tfwrapper as tf
from giskardpy.data_types.exceptions import ObjectForceTorqueThresholdException
from giskardpy.data_types.suturo_types import ForceTorqueThresholds
from giskardpy_ros.python_interface.python_interface import GiskardWrapper


def setup(init_pose_pub: Publisher):
    handle_joint = "iai_kitchen/iai_kitchen:arena:door_handle_joint"
    hinge_joint = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"
    base_pose = PoseStamped()
    base_pose.header.frame_id = 'map'
    base_pose.pose.position.x = 1.5
    base_pose.pose.position.y = -0.8
    base_pose.pose.orientation.z = -1

    odom = gis.monitors.add_local_minimum_reached()
    joint_reset = gis.monitors.add_joint_position(goal_state={handle_joint: 0,
                                                              hinge_joint: 0})
    gis.motion_goals.add_joint_position(goal_state={handle_joint: 0,
                                                    hinge_joint: 0})
    gis.motion_goals.add_cartesian_pose(root_link='map', tip_link='base_footprint', goal_pose=base_pose)

    gis.monitors.add_end_motion(start_condition=f'{joint_reset} and {odom}')
    gis.execute()

    gis.motion_goals.add_take_pose(pose_keyword='park')
    joints = gis.monitors.add_joint_position(goal_state={'head_pan_joint': 0.0,
                                                         'head_tilt_joint': 0.0,
                                                         'arm_lift_joint': 0.0,
                                                         'arm_flex_joint': 0.0,
                                                         'arm_roll_joint': -1.5,
                                                         'wrist_flex_joint': -1.5,
                                                         'wrist_roll_joint': 0.0},
                                             threshold=0.05)
    gis.monitors.add_end_motion(start_condition=joints)
    gis.execute()

    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.pose.pose = base_pose.pose
    init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

    init_pose_pub.publish(init_pose)

    rot_left = QuaternionStamped()
    rot_left.header.frame_id = 'base_footprint'
    rot_left.quaternion.z = 0.643
    rot_left.quaternion.w = 0.766

    rot_right = QuaternionStamped()
    rot_right.header.frame_id = 'base_footprint'
    rot_right.quaternion.z = 0.985
    rot_right.quaternion.w = -0.174

    starting_rot = QuaternionStamped()
    starting_rot.header.frame_id = 'map'
    starting_rot.quaternion.z = -1

    rot_left_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=rot_left,
                                                              root_link='map',
                                                              tip_link='base_footprint',
                                                              name='rotation left monitor')
    rot_right_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=rot_right,
                                                               root_link='map',
                                                               tip_link='base_footprint',
                                                               start_condition=rot_left_monitor,
                                                               name='rotation right monitor')
    rot_start_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=starting_rot,
                                                               root_link='map',
                                                               tip_link='base_footprint',
                                                               start_condition=rot_right_monitor,
                                                               name='rotation start monitor')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=rot_left,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               end_condition=rot_left_monitor,
                                               name='rotation left goal')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=rot_right,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               start_condition=rot_left_monitor,
                                               end_condition=rot_right_monitor,
                                               name='rotation right goal')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=starting_rot,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               start_condition=rot_right_monitor,
                                               end_condition=rot_start_monitor,
                                               name='rotation start goal')

    gis.monitors.add_end_motion(start_condition=rot_start_monitor)
    gis.execute()


def grasping():
    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    hinge_joint = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"
    tip = 'hand_gripper_tool_frame'
    handle_length = 0.01
    ref_speed = 0.3
    handle_retract_distance = 0.077
    bar_center_offset = 0.02
    pre_grasp_distance = 0.15
    grasp_into_distance = -0.1
    ft_timeout = 10000

    bar_axis = Vector3Stamped()
    bar_axis.header.frame_id = handle_name
    bar_axis.vector = Vector3(0, 1, 0)

    tip_grasp_axis = Vector3Stamped()
    tip_grasp_axis.header.frame_id = tip
    tip_grasp_axis.vector = Vector3(1, 0, 0)

    bar_center = PointStamped()
    bar_center.header.frame_id = handle_name
    bar_center.point.y = bar_center_offset

    x_gripper = Vector3Stamped()
    x_gripper.header.frame_id = tip
    x_gripper.vector.z = 1

    x_goal = Vector3Stamped()
    x_goal.header.frame_id = handle_name
    x_goal.vector.z = -1

    pre_grasp = gis.monitors.add_local_minimum_reached(name='pre grasp local min')

    offset_pre = Vector3Stamped()
    offset_pre.header.frame_id = tip
    offset_pre.vector.y = pre_grasp_distance

    gis.motion_goals.add_joint_position(goal_state={hinge_joint: 0})

    gis.motion_goals.hsrb_door_handle_grasp(name='pre grasp', handle_name=handle_name, handle_bar_length=handle_length,
                                            grasp_axis_offset=offset_pre, end_condition=pre_grasp)

    open_gripper = gis.monitors.add_open_hsr_gripper(start_condition=pre_grasp)

    gis.motion_goals.add_align_planes(name='pre grasp align',
                                      tip_link=tip,
                                      tip_normal=x_gripper,
                                      goal_normal=x_goal,
                                      root_link='map',
                                      end_condition=open_gripper)

    gis.motion_goals.add_align_planes(name='grasp align',
                                      tip_link=tip,
                                      tip_normal=x_gripper,
                                      goal_normal=x_goal,
                                      root_link='map',
                                      start_condition=open_gripper)

    offset = Vector3Stamped()
    offset.header.frame_id = tip
    offset.vector.y = grasp_into_distance

    slep = gis.monitors.add_sleep(seconds=ft_timeout, start_condition=open_gripper)
    force = gis.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.DOOR.value, object_type='',
                                          start_condition=open_gripper)
    gis.motion_goals.hsrb_door_handle_grasp(name='grasp', handle_name=handle_name, handle_bar_length=handle_length,
                                            grasp_axis_offset=offset, ref_speed=ref_speed, start_condition=open_gripper,
                                            end_condition=force)

    goal_point = PointStamped()
    goal_point.header.frame_id = 'base_link'

    handle_retract_direction = Vector3Stamped()
    handle_retract_direction.header.frame_id = handle_name
    handle_retract_direction.vector.z = handle_retract_distance

    base_retract = tf.transform_vector(goal_point.header.frame_id, handle_retract_direction)

    goal_point.point = Point(base_retract.vector.x, base_retract.vector.y, base_retract.vector.z)

    gis.motion_goals.add_cartesian_position_straight(root_link='map', tip_link='base_link',
                                                     goal_point=goal_point, start_condition=force)
    grasped = gis.monitors.add_cartesian_position(root_link='map', tip_link='base_link', goal_point=goal_point,
                                                  name='grasped monitor', start_condition=force)

    close_gripper = gis.monitors.add_close_hsr_gripper(start_condition=grasped)

    gis.monitors.add_end_motion(start_condition=close_gripper)
    gis.monitors.add_cancel_motion(f'not {force} and {slep} ',
                                   ObjectForceTorqueThresholdException('Door not touched!'))

    gis.motion_goals.allow_all_collisions()
    gis.execute()


def handle_turning():
    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    handle_joint = "iai_kitchen/iai_kitchen:arena:door_handle_joint"
    hinge_joint = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"

    gis.motion_goals.add_joint_position(goal_state={hinge_joint: 0})
    gis.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame', environment_link=handle_name,
                                        goal_joint_state=0.35)
    handle_monitor = gis.monitors.add_joint_position(goal_state={handle_joint: 0.35})
    gis.monitors.add_end_motion(start_condition=handle_monitor)

    gis.execute()


def hinge_turning():
    hinge_name = 'iai_kitchen/iai_kitchen:arena:door_center'
    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    hinge_joint = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"

    x_goal = Vector3Stamped()
    x_goal.header.frame_id = handle_name
    x_goal.vector.z = -1

    x_base = Vector3Stamped()
    x_base.header.frame_id = 'base_link'
    x_base.vector.y = 1

    gis.motion_goals.add_align_planes(goal_normal=x_goal, tip_link='base_link', tip_normal=x_base, root_link='map')
    gis.motion_goals.add_close_container(tip_link='hand_gripper_tool_frame', environment_link=hinge_name)
    door_hinge_monitor = gis.monitors.add_joint_position(goal_state={hinge_joint: -1.3})
    gis.monitors.add_end_motion(start_condition=door_hinge_monitor)

    gis.execute()


def full_opening():
    grasping()

    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    handle_turn_limit = 0.35
    hinge_turn_limit = -1.3

    gis.motion_goals.hsrb_open_door_goal(door_handle_link=handle_name, handle_limit=handle_turn_limit,
                                         hinge_limit=hinge_turn_limit)

    gis.motion_goals.allow_all_collisions()
    gis.execute()


rospy.init_node('giskard_demo')

init_pub = rospy.Publisher('/initialpose', data_class=PoseWithCovarianceStamped, queue_size=10)

gis = GiskardWrapper()
test = 0

setup(init_pose_pub=init_pub)

# input("Setup finished?")

if test == 1:
    full_opening()
elif test == 2:
    grasping()
elif test == 3:
    handle_turning()
elif test == 4:
    hinge_turning()
else:
    grasping()
    handle_turning()
    hinge_turning()
