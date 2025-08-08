#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped

from giskard_msgs.msg import LinkName
from giskardpy.data_types.suturo_types import TakePoseTypes
from giskardpy_ros.monitors.laser_door_finder import LaserDoorFinder
from giskardpy_ros.python_interface.python_interface import GiskardWrapper

rospy.init_node('demo')

gis = GiskardWrapper()

gis.motion_goals.add_take_pose(pose_keyword=TakePoseTypes.PARK.value)
gis.add_default_end_motion_conditions()
gis.execute()

base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position.y = -1
base_goal.pose.orientation.w = 1

pre_laser = gis.monitors.add_cartesian_pose(root_link='map', tip_link='base_link', goal_pose=base_goal)

gis.motion_goals.add_cartesian_pose(goal_pose=base_goal, root_link='map', tip_link='base_link', end_condition=pre_laser)

finder = gis.monitors.add_monitor(class_name=LaserDoorFinder.__name__,
                                  vector_topic='door_vector',
                                  point_topic='door_points',
                                  door_hinge_name=LinkName(group_name='iai_kitchen', name='iai_kitchen:arena:door_hinge'),
                                  door_handle_name=LinkName(group_name='iai_kitchen', name='iai_kitchen:arena:door_handle_link'),
                                  door_main_name=LinkName(group_name='iai_kitchen', name='iai_kitchen:arena:door_center'),
                                  start_condition=pre_laser)

gis.monitors.add_end_motion(start_condition=finder)

gis.execute()

handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_outside"
hinge_joint = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"
door_handle_parent_link = 'iai_kitchen/iai_kitchen:arena:door_center'

open_gripper = gis.monitors.add_open_hsr_gripper()

x_gripper = Vector3Stamped()
x_gripper.header.frame_id = 'hand_gripper_tool_frame'
x_gripper.vector.z = 1

x_goal = Vector3Stamped()
x_goal.header.frame_id = handle_name
x_goal.vector.z = -1

gis.motion_goals.add_align_planes(tip_link='hand_gripper_tool_frame',
                                  tip_normal=x_gripper,
                                  goal_normal=x_goal,
                                  root_link='map', start_condition=open_gripper)

gis.motion_goals.hsrb_door_handle_grasp(handle_name=handle_name, handle_bar_length=0.05, start_condition=open_gripper)

local_min = gis.monitors.add_local_minimum_reached(start_condition=open_gripper)
gis.monitors.add_end_motion(start_condition=local_min)

gis.execute()

close = gis.monitors.add_close_hsr_gripper()

gis.motion_goals.add_close_container(tip_link='hand_gripper_tool_frame',
                                     environment_link=door_handle_parent_link,
                                     goal_joint_state=0, start_condition=close)
joint_monitor = gis.monitors.add_joint_position(goal_state={hinge_joint: 0}, start_condition=close)
gis.monitors.add_end_motion(start_condition=joint_monitor)

gis.execute()
