import csv
import os.path
from typing import Dict

import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
from geometry_msgs.msg import WrenchStamped
from py_trees import Status

from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils import logging
from giskardpy.utils.decorators import catch_and_raise_to_blackboard

# data extraction
from numpy import savetxt
from numpy import asarray
from pprint import pprint

import scipy
# !/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, lfilter


class ButterworthFilter:
    def __init__(self, cutoff, fs, order=4):
        self.prev_values = [WrenchStamped()] * (order + 1)
        self.b, self.a = butter(order, cutoff / (0.5 * fs), btype='low')

    def filter(self, current_input):
        filtered_data = WrenchStamped()
        filtered_data.header = current_input.header

        # Apply filter for each force and torque component
        for attr in ['x', 'y', 'z']:
            force_values = [getattr(val.wrench.force, attr) for val in self.prev_values] + [
                getattr(current_input.wrench.force, attr)]
            torque_values = [getattr(val.wrench.torque, attr) for val in self.prev_values] + [
                getattr(current_input.wrench.torque, attr)]

            filtered_force = lfilter(self.b, self.a, force_values)[-1]
            filtered_torque = lfilter(self.b, self.a, torque_values)[-1]

            setattr(filtered_data.wrench.force, attr, filtered_force)
            setattr(filtered_data.wrench.torque, attr, filtered_torque)

        self.prev_values.append(current_input)
        self.prev_values.pop(0)

        return filtered_data


def callback(data, filter_obj, pub):
    filtered_data = filter_obj.filter(data)
    pub.publish(filtered_data)


def main():
    rospy.init_node('butterworth_filter_node', anonymous=True)

    # Sampling frequency (for a 60Hz publisher)
    fs = 60.0
    # Customize these values as needed:
    cutoff = rospy.get_param("~cutoff", 10.0)  # Cutoff frequency in Hz
    order = rospy.get_param("~order", 4)  # Order of the filter

    filter_obj = ButterworthFilter(cutoff, fs, order)

    pub = rospy.Publisher('/filtered_force_torque', WrenchStamped, queue_size=10)
    rospy.Subscriber('/raw_force_torque', WrenchStamped, callback, filter_obj, pub)

    rospy.spin()


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name, conditions, recovery):
        super().__init__(name)
        self.arm_trajectory_publisher = None
        self.cancel_condition = False
        self.name = name
        self.wrench_compensated_subscriber = None

        self.filter_range = 4
        self.wrench_compensated_force_data_x = [0] * self.filter_range
        self.wrench_compensated_force_data_y = [0] * self.filter_range
        self.wrench_compensated_force_data_z = [0] * self.filter_range
        self.wrench_compensated_torque_data_x = [0] * self.filter_range
        self.wrench_compensated_torque_data_y = [0] * self.filter_range
        self.wrench_compensated_torque_data_z = [0] * self.filter_range
        self.wrench_compensated_latest_data = WrenchStamped()
        self.sensor_timestamps = [0] * self.filter_range
        self.sensor_seq = [0] * self.filter_range

        self.x_force_filtered = []
        self.y_force_filtered = []
        self.z_force_filtered = []
        self.x_torque_filtered = []
        self.y_torque_filtered = []
        self.z_torque_filtered = []

        # self.force_threshold = 0.0
        # self.torque_threshold = 0.15
        # self.force_derivative_threshold = 50
        # self.force_derivative_threshold = 50
        self.conditions = conditions

        self.counter = 0

        self.recovery = recovery

        # True to print sensor data
        self.show_data = False

    @profile
    def setup(self, timeout):
        self.wrench_compensated_subscriber = rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped,
                                                              self.get_rospy_data)

        # initialize ROS publisher
        self.arm_trajectory_publisher = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                                                        trajectory_msgs.msg.JointTrajectory, queue_size=10)
        # wait to establish connection between the controller
        while self.arm_trajectory_publisher.get_num_connections() == 0:
            rospy.sleep(0.1)

        # make sure the controller is running

        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = (
            rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                               controller_manager_msgs.srv.ListControllers))
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True

        print('running')

        return True

    @profile
    def get_rospy_data(self,
                       data_compensated: WrenchStamped):

        self.add_data(data_compensated)

        self.cancel_goal_check()

        if self.cancel_condition:
            self.wrench_compensated_subscriber.unregister()

            print('unsubscribed')

        if self.show_data:
            print(data_compensated.wrench.force)

    def cancel_goal_check(self):
        """
        place with frontal grasping: force: -x, torque: y
        """

        # TODO: improve math (e.g. work with derivative or moving average)

        # if self.counter > 2:
        #    self.cancel_condition = True

        filtered_data = {'x_force': self.x_force_filtered,
                         'y_force': self.y_force_filtered,
                         'z_force': self.z_force_filtered,
                         'x_torque': self.x_torque_filtered,
                         'y_torque': self.y_torque_filtered,
                         'z_torque': self.z_torque_filtered}

        conds = self.conditions
        evals = []
        for condition in conds:
            sensor_axis, operator, value = condition

            current_data = filtered_data[sensor_axis][-1]

            evaluated_condition = eval(f'{current_data} {operator} {value}')

            evals.append(evaluated_condition)

        if any(evals):
            logging.loginfo(f'conditions: {conds}')
            logging.loginfo(f'evaluated: {evals}')

            self.cancel_condition = True

    def add_data(self,
                 data_compensated: WrenchStamped):
        self.wrench_compensated_force_data_x.append(data_compensated.wrench.force.x)
        self.wrench_compensated_force_data_y.append(data_compensated.wrench.force.y)
        self.wrench_compensated_force_data_z.append(data_compensated.wrench.force.z)

        self.wrench_compensated_torque_data_x.append(data_compensated.wrench.torque.x)
        self.wrench_compensated_torque_data_y.append(data_compensated.wrench.torque.y)
        self.wrench_compensated_torque_data_z.append(data_compensated.wrench.torque.z)

        self.wrench_compensated_latest_data = data_compensated
        self.sensor_timestamps.append(self.wrench_compensated_latest_data.header.stamp)
        self.sensor_seq.append(self.wrench_compensated_latest_data.header.seq)

        order = self.filter_range
        cutoff = 10
        fs = 60
        b, a = butter(order, cutoff / (0.5 * fs), btype='low')

        # filter_range = self.filter_range

        current_x_force_filtered = lfilter(b, a, self.wrench_compensated_force_data_x)[-1]
        current_y_force_filtered = lfilter(b, a, self.wrench_compensated_force_data_y)[-1]
        current_z_force_filtered = lfilter(b, a, self.wrench_compensated_force_data_z)[-1]
        current_x_torque_filtered = lfilter(b, a, self.wrench_compensated_torque_data_x)[-1]
        current_y_torque_filtered = lfilter(b, a, self.wrench_compensated_torque_data_y)[-1]
        current_z_torque_filtered = lfilter(b, a, self.wrench_compensated_torque_data_z)[-1]

        self.x_force_filtered.append(current_x_force_filtered)
        self.y_force_filtered.append(current_y_force_filtered)
        self.z_force_filtered.append(current_z_force_filtered)
        self.x_torque_filtered.append(current_x_torque_filtered)
        self.y_torque_filtered.append(current_y_torque_filtered)
        self.z_torque_filtered.append(current_z_torque_filtered)

    @catch_and_raise_to_blackboard
    @profile
    def update(self):

        if self.cancel_condition:
            self.save_data()
            rospy.loginfo('goal canceled')

            return Status.SUCCESS
            # raise MonitorForceException

        self.counter += 1

        return Status.FAILURE

    def terminate(self, new_status):

        if self.cancel_condition:
            self.recover()
            tree = self.tree
            tree.remove_node(self.name)

    def recover(self):

        joint_names = ['arm_lift_joint', 'arm_flex_joint',
                       'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

        joint_modify: Dict = self.recovery

        joint_positions = []
        for joint_name in joint_names:
            join_state_position = self.world.state.get(self.world.search_for_joint_name(joint_name)).position

            if joint_name in joint_modify:
                mod = joint_modify.get(joint_name)
            else:
                mod = 0.0

            joint_positions.append(join_state_position + mod)

        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = joint_names

        trajectory_point = trajectory_msgs.msg.JointTrajectoryPoint()
        trajectory_point.positions = joint_positions
        trajectory_point.velocities = [0, 0, 0, 0, 0]
        trajectory_point.time_from_start = rospy.Duration(1)
        traj.points = [trajectory_point]

        # publish ROS message
        self.arm_trajectory_publisher.publish(traj)

    def save_data(self):

        filtered_data = {'timestamp': self.sensor_timestamps,
                         'seq': self.sensor_seq,
                         'x_force': self.x_force_filtered,
                         'y_force': self.y_force_filtered,
                         'z_force': self.z_force_filtered,
                         'x_torque': self.x_torque_filtered,
                         'y_torque': self.y_torque_filtered,
                         'z_torque': self.z_torque_filtered,
                         }

        unfiltered_data = {'timestamp': self.sensor_timestamps,
                           'seq': self.sensor_seq,
                           'x_force': self.wrench_compensated_force_data_x,
                           'y_force': self.wrench_compensated_force_data_y,
                           'z_force': self.wrench_compensated_force_data_z,
                           'x_torque': self.wrench_compensated_torque_data_x,
                           'y_torque': self.wrench_compensated_torque_data_y,
                           'z_torque': self.wrench_compensated_torque_data_z,
                           }

        pprint(filtered_data)
        pprint(unfiltered_data)

        with open(os.path.expanduser('~/ForceTorqueData/filtered.csv'), 'w') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(filtered_data.keys())

            for index, value in enumerate(self.x_force_filtered):
                filtered_row = [filtered_data['timestamp'][index], filtered_data['seq'][index],
                                filtered_data['x_force'][index], filtered_data['y_force'][index],
                                filtered_data['z_force'][index], filtered_data['x_torque'][index],
                                filtered_data['y_torque'][index], filtered_data['z_torque'][index]]
                writer.writerow(filtered_row)

        with open(os.path.expanduser('~/ForceTorqueData/unfiltered.csv'), 'w') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(unfiltered_data.keys())

            for index, value in enumerate(self.wrench_compensated_force_data_x):
                unfiltered_row = [unfiltered_data['timestamp'][index], unfiltered_data['seq'][index],
                                  unfiltered_data['x_force'][index], unfiltered_data['y_force'][index],
                                  unfiltered_data['z_force'][index], unfiltered_data['x_torque'][index],
                                  unfiltered_data['y_torque'][index], unfiltered_data['z_torque'][index]]
                writer.writerow(unfiltered_row)
