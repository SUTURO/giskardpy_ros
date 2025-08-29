import rospy
from geometry_msgs.msg import PoseStamped, Point, Vector3Stamped, PointStamped

from giskardpy.data_types.suturo_types import TakePoseTypes
from giskardpy_ros.python_interface.python_interface import GiskardWrapper

rospy.init_node(name='fridge_demo')

kitchen_setup = GiskardWrapper()

handle_frame_id = 'iai_kitchen/iai_fridge_door_handle'
handle_name = 'iai_fridge_door_handle'
kitchen_setup.monitors.add_open_hsr_gripper()
base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position = Point(2.0, -1.0, 0.0)
base_goal.pose.orientation.z = -0.707
base_goal.pose.orientation.w = 0.707
kitchen_setup.motion_goals.add_cartesian_pose(goal_pose=base_goal, tip_link='base_footprint', root_link='map')
kitchen_setup.motion_goals.add_take_pose(pose_keyword=TakePoseTypes.PARK.value)
kitchen_setup.add_default_end_motion_conditions()
kitchen_setup.motion_goals.allow_all_collisions()
kitchen_setup.execute()

bar_axis = Vector3Stamped()
bar_axis.header.frame_id = handle_frame_id
bar_axis.vector.z = 1

bar_center = PointStamped()
bar_center.header.frame_id = handle_frame_id

tip_grasp_axis = Vector3Stamped()
tip_grasp_axis.header.frame_id = 'hand_gripper_tool_frame'
tip_grasp_axis.vector.x = 1

kitchen_setup.motion_goals.add_grasp_bar(root_link='map',
                                         tip_link='hand_gripper_tool_frame',
                                         tip_grasp_axis=tip_grasp_axis,
                                         bar_center=bar_center,
                                         bar_axis=bar_axis,
                                         bar_length=.4)
x_gripper = Vector3Stamped()
x_gripper.header.frame_id = 'hand_gripper_tool_frame'
x_gripper.vector.z = 1

x_goal = Vector3Stamped()
x_goal.header.frame_id = handle_frame_id
x_goal.vector.x = -1
kitchen_setup.motion_goals.add_align_planes(tip_link='hand_gripper_tool_frame',
                                            tip_normal=x_gripper,
                                            goal_normal=x_goal,
                                            root_link='map')
kitchen_setup.motion_goals.allow_all_collisions()
kitchen_setup.add_default_end_motion_conditions()
kitchen_setup.execute()

kitchen_setup.monitors.add_close_hsr_gripper()

kitchen_setup.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame',
                                              environment_link=handle_name,
                                              goal_joint_state=1.5)

kitchen_setup.motion_goals.allow_all_collisions()
kitchen_setup.add_default_end_motion_conditions()
kitchen_setup.execute()

open = kitchen_setup.monitors.add_open_hsr_gripper()
kitchen_setup.monitors.add_end_motion(start_condition=open)
kitchen_setup.execute()