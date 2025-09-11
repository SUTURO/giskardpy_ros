#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3Stamped, Point, Quaternion

from giskardpy.data_types.suturo_types import ForceTorqueThresholds, TakePoseTypes
from giskardpy.utils.math import quaternion_from_axis_angle
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros1 import tfwrapper as tf


def setup(base_pose, hinge_joint, takepose: str):
    odom = gis.monitors.add_local_minimum_reached()
    gis.monitors.add_open_hsr_gripper()
    joint_reset = gis.monitors.add_joint_position(goal_state={hinge_joint: 0})

    gis.motion_goals.add_joint_position(goal_state={hinge_joint: 0}, name='door hinge monitor')
    gis.motion_goals.add_cartesian_pose(root_link='map', tip_link='base_footprint', goal_pose=base_pose)

    gis.motion_goals.add_take_pose(pose_keyword=takepose)
    if takepose == TakePoseTypes.PARK_LEFT.value:
        joints = gis.monitors.add_joint_position(goal_state={'head_pan_joint': 0.0,
                                                             'head_tilt_joint': 0.0,
                                                             'arm_lift_joint': 0.0,
                                                             'arm_flex_joint': 0.0,
                                                             'arm_roll_joint': 1.5,
                                                             'wrist_flex_joint': -1.5,
                                                             'wrist_roll_joint': 0.0},
                                                 threshold=0.05,
                                                 name='park arms monitor')
    else:
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
    gis.motion_goals.allow_all_collisions()
    gis.execute()


rospy.init_node('giskard_demo')

gis = GiskardWrapper()

vertical_grasp = True

handle_left_id = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_left:handle'
hinge_left_joint = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_left:joint'
handle_right_id = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_right:handle'
hinge_joint_right = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_right:joint'

setup_pose = PoseStamped()
setup_pose.header.frame_id = 'map'
setup_pose.pose.position.x = 4.3
setup_pose.pose.position.y = 4.0
setup_pose.pose.orientation = Quaternion(*quaternion_from_axis_angle(axis=(0, 0, 1), angle=0))

setup(setup_pose, hinge_joint_right, TakePoseTypes.PARK.value)

gis.billy_shelf_open(setup_pose=setup_pose, simulation=True, open_left=True, open_right=True)
