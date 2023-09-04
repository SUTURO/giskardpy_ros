import csv
import math
import os.path
from typing import Dict

import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
from geometry_msgs.msg import WrenchStamped
from py_trees import Status

from giskardpy.exceptions import GiskardException
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils import logging
from giskardpy.utils.decorators import catch_and_raise_to_blackboard
from giskardpy import casadi_wrapper as w
# data extraction
from numpy import savetxt
from numpy import asarray
from pprint import pprint

import scipy
# !/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, lfilter


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name, condition, recovery):
        super().__init__(name)

        self.name = name
        self.robot_name = self.world.robot_name

        self.condition = condition
        self.recovery = recovery
        self.cancel_condition = False

        # Data
        self.whole_data = {'unfiltered': [],
                           'filtered': []}

        # Filter
        self.prev_values = None
        self.order = 4
        cutoff = 10
        fs = 60
        self.b, self.a = butter(self.order, cutoff / (0.5 * fs), btype='low')

        # Subscriber
        self.wrench_compensated_subscriber = None

        # Publisher
        self.base_pub = None
        self.arm_trajectory_publisher = None

        if self.control_mode == self.control_mode.open_loop:
            self.continue_plugin_state = Status.FAILURE
        elif self.control_mode == self.control_mode.close_loop:
            self.continue_plugin_state = Status.RUNNING
        else:
            logging.logerr(f'{self.control_mode} is not supported')
            raise GiskardException()

        # True to print sensor data
        self.show_data = False
        self.cancel = True
        self.plugin_canceled = (False, 0)

        self.init_data = True

    @profile
    def setup(self, timeout):

        if self.robot_name == 'hsrb':
            controller_list_topic = '/hsrb/controller_manager/list_controllers'
            wrench_topic = '/hsrb/wrist_wrench/compensated'
            arm_trajectory_topic = '/hsrb/arm_trajectory_controller/command'
            base_trajectory_topic = '/hsrb/omni_base_controller/command'

        elif self.robot_name == 'iai_donbot':
            controller_list_topic = None
            wrench_topic = '/kms40_driver/wrench'
            arm_trajectory_topic = '/scaled_pos_joint_traj_controller/command'
            base_trajectory_topic = None  # Does not work yet: '/whole_body_controller/base'

        else:
            logging.logerr(f'{self.robot_name} is not supported')
            raise GiskardException()

        self.wrench_compensated_subscriber = rospy.Subscriber(wrench_topic, WrenchStamped,
                                                              self.get_rospy_data)

        if self.control_mode == self.control_mode.open_loop:
            # initialize ROS publisher
            if arm_trajectory_topic is not None:
                self.arm_trajectory_publisher = rospy.Publisher(arm_trajectory_topic,
                                                                trajectory_msgs.msg.JointTrajectory, queue_size=10)

                # wait to establish connection between the controller
                while self.arm_trajectory_publisher.get_num_connections() == 0:
                    logging.logwarn(f'connecting to {arm_trajectory_topic}')
                    rospy.sleep(0.1)

            if base_trajectory_topic is not None:
                self.base_pub = rospy.Publisher(base_trajectory_topic,
                                                trajectory_msgs.msg.JointTrajectory, queue_size=10)

                # wait to establish connection between the controller
                while self.base_pub.get_num_connections() == 0:
                    logging.logwarn(f'connecting to {base_trajectory_topic}')
                    rospy.sleep(0.1)

            # make sure the controller is running
            if controller_list_topic is not None:
                rospy.wait_for_service(controller_list_topic)
                list_controllers = (
                    rospy.ServiceProxy(controller_list_topic,
                                       controller_manager_msgs.srv.ListControllers))

                running1, running2 = False, False
                while running1 is False or running2 is False:
                    rospy.sleep(0.1)
                    for c in list_controllers().controller:
                        if c.name == 'arm_trajectory_controller' and c.state == 'running':
                            running1 = True
                        if c.name == 'omni_base_controller' and c.state == 'running':
                            running2 = True

                print('running')

        return True

    @profile
    def get_rospy_data(self,
                       data_compensated: WrenchStamped):
        if self.init_data:
            self.prev_values = [data_compensated] * (self.order + 1)
            self.init_data = False

        self.add_data(data_compensated)

        self.cancel_goal_check()

        if self.cancel_condition:
            self.wrench_compensated_subscriber.unregister()

            print('unsubscribed')

        if self.show_data:
            print(data_compensated.wrench.force)

    def add_data(self,
                 data_compensated: WrenchStamped):

        filtered_data = WrenchStamped()
        filtered_data.header = data_compensated.header

        # Apply filter for each force and torque component
        for attr in ['x', 'y', 'z']:
            force_values = [getattr(val.wrench.force, attr) for val in self.prev_values] + [
                getattr(data_compensated.wrench.force, attr)]
            torque_values = [getattr(val.wrench.torque, attr) for val in self.prev_values] + [
                getattr(data_compensated.wrench.torque, attr)]

            filtered_force = lfilter(self.b, self.a, force_values)[-1]
            filtered_torque = lfilter(self.b, self.a, torque_values)[-1]

            setattr(filtered_data.wrench.force, attr, filtered_force)
            setattr(filtered_data.wrench.torque, attr, filtered_torque)

        self.prev_values.append(data_compensated)
        self.prev_values.pop(0)

        self.whole_data['unfiltered'].append(data_compensated)
        self.whole_data['filtered'].append(filtered_data)

    def cancel_goal_check(self):
        """
        place with frontal grasping: force: -x, torque: y
        """

        filtered_data = self.whole_data['filtered'][-1]

        filtered_dict = {'x_force': filtered_data.wrench.force.x,
                         'y_force': filtered_data.wrench.force.y,
                         'z_force': filtered_data.wrench.force.z,
                         'x_torque': filtered_data.wrench.torque.x,
                         'y_torque': filtered_data.wrench.torque.y,
                         'z_torque': filtered_data.wrench.torque.z}


        # FIXME: condition for norm (np.linag.norm)

        condition_triggered = self.condition(filtered_dict)

        if condition_triggered:
            self.plugin_canceled = (True, filtered_data.header.stamp, filtered_data.header.seq)

            if self.cancel:
                logging.loginfo(f'tree cycle:  {self.tree_manager.tree.count}')

                self.cancel_condition = True


    @catch_and_raise_to_blackboard
    @profile
    def update(self):

        self.save_data()

        if self.cancel_condition:

            logging.loginfo('goal canceled')

            raise GiskardException()

        return self.continue_plugin_state


    def recover(self):

        joint_modify: Dict = self.recovery

        arm_joint_names = ['arm_lift_joint', 'arm_flex_joint',
                           'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

        odom_joint_names = ["odom_x", "odom_y", "odom_t"]

        if any(x in joint_modify for x in arm_joint_names):
            arm_joint_positions = []
            for joint_name in arm_joint_names:
                join_state_position = self.world.state.get(self.world.search_for_joint_name(joint_name)).position

                if joint_name in joint_modify:
                    mod = joint_modify.get(joint_name)
                else:
                    mod = 0.0

                arm_joint_positions.append(join_state_position + mod)

            # fill ROS message
            traj = trajectory_msgs.msg.JointTrajectory()
            traj.joint_names = arm_joint_names

            trajectory_point = trajectory_msgs.msg.JointTrajectoryPoint()
            trajectory_point.positions = arm_joint_positions
            trajectory_point.velocities = [0, 0, 0, 0, 0]
            trajectory_point.time_from_start = rospy.Duration(1)
            traj.points = [trajectory_point]

            # publish ROS message
            self.arm_trajectory_publisher.publish(traj)

        if any(x in joint_modify for x in odom_joint_names):
            c_T_o = w.TransMatrix(
                self.god_map.evaluate_expr(self.world.joints['hsrb/brumbrum'].parent_T_child)).to_position()

            odom_joint_positions = c_T_o.compile().fast_call(
                self.god_map.get_values(c_T_o.compile().str_params)).tolist()[:3]

            odom_positions = []
            for index, names in enumerate(odom_joint_names):
                if names in joint_modify:
                    mod = joint_modify.get(names)
                else:
                    mod = 0.0
                odom_positions.append(odom_joint_positions[index] + mod)

            # Send data
            traj = trajectory_msgs.msg.JointTrajectory()
            traj.joint_names = odom_joint_names
            p = trajectory_msgs.msg.JointTrajectoryPoint()
            p.positions = odom_positions
            p.velocities = [0, 0, 0]
            p.time_from_start = rospy.Duration(5)
            traj.points = [p]

            # publish ROS message
            self.base_pub.publish(traj)

    def save_data(self):

        types = ['filtered', 'unfiltered']
        keys = ['timestamp', 'seq', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']
        standard_path = '~/SUTURO/SUTURO_WSS/manipulation_ws/src/suturo_manipulation/suturo_manipulation/src/suturo_manipulation/'

        for current_type in types:
            data = [[ws.header.stamp, ws.header.seq,
                     ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z,
                     ws.wrench.torque.x, ws.wrench.torque.y, ws.wrench.torque.z]
                    for ws in self.whole_data[current_type]]


            with open(os.path.expanduser(standard_path + current_type + '.csv'), 'w') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(keys + [self.plugin_canceled])

                for row_value in data:
                    writer.writerow(row_value)
