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


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name, conditions, recovery):
        super().__init__(name)
        self.arm_trajectory_publisher = None
        self.cancel_condition = False
        self.name = name
        self.wrench_compensated_subscriber = None

        order = 4
        cutoff = 10
        fs = 60
        self.b, self.a = butter(order, cutoff / (0.5 * fs), btype='low')

        self.whole_data = {'unfiltered': [],
                           'filtered': []}

        self.prev_values = [WrenchStamped()] * (order + 1)

        self.conditions = conditions

        self.recovery = recovery

        # True to print sensor data
        self.show_data = False
        self.plugin_canceled = (False, 0)


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

        filtered_data = self.whole_data['filtered'][-1]

        filtered_dict = {'x_force': filtered_data.wrench.force.x,
                         'y_force': filtered_data.wrench.force.y,
                         'z_force': filtered_data.wrench.force.z,
                         'x_torque': filtered_data.wrench.torque.x,
                         'y_torque': filtered_data.wrench.torque.y,
                         'z_torque': filtered_data.wrench.torque.z}

        conds = self.conditions
        evals = []
        for condition in conds:
            sensor_axis, operator, value = condition

            current_data = filtered_dict[sensor_axis]

            evaluated_condition = eval(f'{current_data} {operator} {value}')

            evals.append(evaluated_condition)

        if any(evals):
            logging.loginfo(f'conditions: {conds}')
            logging.loginfo(f'evaluated: {evals}')

            self.plugin_canceled = (True, filtered_data.header.stamp, filtered_data.header.seq)

            self.cancel_condition = True

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

    @catch_and_raise_to_blackboard
    @profile
    def update(self):

        self.save_data()

        if self.cancel_condition:
            rospy.loginfo('goal canceled')

            return Status.SUCCESS
            # raise MonitorForceException

        # self.counter += 1

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

        types = ['filtered', 'unfiltered']
        keys = ['timestamp', 'seq', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']
        path = '~/SUTURO/SUTURO_WSS/manipulation_ws/src/suturo_manipulation/suturo_manipulation/src/suturo_manipulation/'

        for current_type in types:
            data = [[ws.header.stamp, ws.header.seq,
                     ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z,
                     ws.wrench.torque.x, ws.wrench.torque.y, ws.wrench.torque.z]
                    for ws in self.whole_data[current_type]]

            # pprint(data)

            with open(os.path.expanduser(path + current_type + '.csv'), 'w') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(keys + [self.plugin_canceled])

                for row_value in data:
                    writer.writerow(row_value)

