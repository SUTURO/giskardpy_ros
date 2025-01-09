#!/usr/bin/env python
from copy import deepcopy
from typing import Dict

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, Point
from tmc_control_msgs.msg import GripperApplyEffortActionResult

from giskardpy_ros.python_interface.python_interface import GiskardWrapper


class Demo:
    def run(self):
        open_drawers = []

        rospy.init_node('giskardpy_shelf_opening')

        self.echo = rospy.Publisher('/hsrb/gripper_controller/grasp/result', GripperApplyEffortActionResult,
                                    queue_size=1)

        self.gis = GiskardWrapper()

        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        # base_goal.pose.position = Point(4.4, -1, 0)
        # base_goal.pose.orientation.z = -0.7042499909504866
        base_goal.pose.orientation.z = 0.7042499909504866
        base_goal.pose.orientation.w = 0.709952076020797

        # handle_frame_id_upper = 'iai_kitchen/oven_area_area_middle_upper_drawer_handle'
        # handle_name_upper = 'oven_area_area_middle_upper_drawer_handle'
        # drawer_fix_upper = {'oven_area_area_middle_upper_drawer_main_joint': 0}
        #
        # open_drawers.append((deepcopy(base_goal), handle_frame_id_upper, handle_name_upper, drawer_fix_upper))

        # base_goal.pose.position = Point(2, -1, 0)
        #
        # handle_frame_id_pre = 'iai_kitchen/sink_area_trash_drawer_handle'
        # handle_name_pre = 'sink_area_trash_drawer_handle'
        # drawer_fix_dict = {'sink_area_trash_drawer_main_joint': 0}

        # base_goal.pose.position = Point(3.5, -1, 0)
        #
        # handle_frame_id_pre = 'iai_kitchen/sink_area_left_bottom_drawer_handle'
        # handle_name_pre = 'sink_area_left_bottom_drawer_handle'
        # drawer_fix_dict = {'sink_area_left_bottom_drawer_main_joint': 0}

        # base_goal.pose.position = Point(4, -1, 0)
        #
        # handle_frame_id_pre = 'iai_kitchen/oven_area_area_right_drawer_handle'
        # handle_name_pre = 'oven_area_area_right_drawer_handle'
        # drawer_fix_dict = {'oven_area_area_right_drawer_joint': 0}

        base_goal.pose.position = Point(4, -1, 0)

        handle_frame_id_pre = 'iai_kitchen/kitchen_island_middle_lower_drawer_handle'
        handle_name_pre = 'kitchen_island_middle_lower_drawer_handle'
        drawer_fix_dict = {'kitchen_island_middle_lower_drawer_main_joint': 0}

        open_drawers.append((deepcopy(base_goal), handle_frame_id_pre, handle_name_pre, drawer_fix_dict))

        for goal, handle_frame_id, handle_name, drawer_fix in open_drawers:
            self.open_sequence(goal, handle_name, handle_frame_id, drawer_fix)
            # self.close_sequence(handle_name, handle_frame_id)

    def open_sequence(self,
                      goal: PoseStamped,
                      handle_name: str,
                      handle_frame_id: str,
                      drawer_fix: Dict[str, float]):
        # self.open_gripper()

        self.gis.motion_goals.allow_all_collisions()
        self.gis.motion_goals.add_take_pose('park')
        monitor = self.gis.monitors.add_cartesian_pose(goal_pose=goal, tip_link='base_footprint', root_link='map',
                                                       name='base goal')
        self.gis.motion_goals.add_cartesian_pose(goal_pose=goal, tip_link='base_footprint', root_link='map',
                                                 name='base goal',
                                                 end_condition=monitor)
        local_min = self.gis.monitors.add_local_minimum_reached('local_min', start_condition=monitor)
        self.gis.monitors.add_end_motion(start_condition=local_min)
        self.gis.execute()

        open_gripper = self.gis.monitors.add_open_hsr_gripper(name='open gripper start')
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = -1
        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id
        bar_center.point.x = -0.03  # Offset for Grasping
        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = 'hand_gripper_tool_frame'
        tip_grasp_axis.vector.x = 1
        # grasping motion
        bar_grasped = self.gis.monitors.add_distance_to_line(name='bar grasped',
                                                             root_link='map',
                                                             tip_link='hand_gripper_tool_frame',
                                                             center_point=bar_center,
                                                             line_axis=bar_axis,
                                                             line_length=.4,
                                                             start_condition=open_gripper)
        self.gis.motion_goals.add_grasp_bar(root_link='map',
                                            tip_link='hand_gripper_tool_frame',
                                            tip_grasp_axis=tip_grasp_axis,
                                            bar_center=bar_center,
                                            bar_axis=bar_axis,
                                            bar_length=.4,
                                            name='grasp bar',
                                            start_condition=open_gripper,
                                            end_condition=bar_grasped)
        self.gis.motion_goals.add_joint_position(goal_state=drawer_fix,
                                                 end_condition=bar_grasped)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = 'hand_gripper_tool_frame'
        x_gripper.vector.z = 1
        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        self.gis.motion_goals.add_align_planes(tip_link='hand_gripper_tool_frame',
                                               tip_normal=x_gripper,
                                               goal_normal=x_goal,
                                               root_link='map',
                                               name='orient to door',
                                               start_condition=open_gripper,
                                               end_condition=bar_grasped)
        close_gripper = self.gis.monitors.add_close_hsr_gripper(start_condition=bar_grasped)
        sleep = self.gis.monitors.add_sleep(seconds=0.5, start_condition=close_gripper)

        # open container
        door_open = self.gis.monitors.add_local_minimum_reached(name='door open',
                                                                start_condition=sleep)
        self.gis.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame',
                                                 environment_link=handle_name,
                                                 goal_joint_state=0.46,
                                                 name='open door',
                                                 start_condition=sleep,
                                                 end_condition=door_open)

        open_gripper_end = self.gis.monitors.add_open_hsr_gripper(name='open gripper end', start_condition=door_open)

        self.gis.motion_goals.allow_all_collisions()
        self.gis.monitors.add_end_motion(start_condition=open_gripper_end)
        self.gis.execute()

        # self.close_gripper()

    def close_sequence(self,
                       handle_name: str,
                       handle_frame_id: str):
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = -1
        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id
        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = 'hand_gripper_tool_frame'
        tip_grasp_axis.vector.x = 1
        # %% phase 1
        bar_grasped = self.gis.monitors.add_distance_to_line(name='bar grasped',
                                                             root_link='map',
                                                             tip_link='hand_gripper_tool_frame',
                                                             center_point=bar_center,
                                                             line_axis=bar_axis,
                                                             line_length=.4)
        self.gis.motion_goals.add_grasp_bar(root_link='map',
                                            tip_link='hand_gripper_tool_frame',
                                            tip_grasp_axis=tip_grasp_axis,
                                            bar_center=bar_center,
                                            bar_axis=bar_axis,
                                            bar_length=.4,
                                            name='grasp bar',
                                            end_condition=bar_grasped)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = 'hand_gripper_tool_frame'
        x_gripper.vector.z = 1
        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        self.gis.motion_goals.add_align_planes(tip_link='hand_gripper_tool_frame',
                                               tip_normal=x_gripper,
                                               goal_normal=x_goal,
                                               root_link='map',
                                               name='orient to door',
                                               end_condition=bar_grasped)
        # %% phase 2 close door
        door_close = self.gis.monitors.add_local_minimum_reached(name='door close',
                                                                 start_condition=bar_grasped)
        self.gis.motion_goals.add_close_container(tip_link='hand_gripper_tool_frame',
                                                  environment_link=handle_name,
                                                  name='open close',
                                                  start_condition=bar_grasped,
                                                  end_condition=door_close)

        open_gripper = self.gis.monitors.add_open_hsr_gripper(start_condition=door_close)

        self.gis.motion_goals.allow_all_collisions()
        self.gis.monitors.add_end_motion(start_condition=open_gripper)
        self.gis.execute()

        # self.open_gripper()

    def open_gripper(self):
        self.command_gripper(1.23)

    def close_gripper(self):
        self.command_gripper(0)

    def command_gripper(self, width):
        js = {'hand_motor_joint': width}
        end_condition = self.gis.monitors.add_joint_position(goal_state=js)
        self.gis.motion_goals.add_joint_position(end_condition=end_condition,
                                                 goal_state=js)
        self.gis.motion_goals.allow_all_collisions()
        self.gis.monitors.add_end_motion(start_condition=end_condition)
        self.gis.execute()


if __name__ == '__main__':
    demo = Demo()
    demo.run()
