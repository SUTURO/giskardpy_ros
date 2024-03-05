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
from giskardpy.god_map import god_map
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

import rospkg
import rospy
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, lfilter


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name, condition, wrench_topic):
        super().__init__(name)

        self.name = name
        self.rospack = rospkg.RosPack()

        self.condition = condition
        self.wrench_topic = wrench_topic
        self.cancel_condition = False

        # Data
        self.filtered_data = {}
        self.filtered_derivatives = {}
        self.current_data = {}
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

        if god_map.tree.control_mode == god_map.tree.control_mode.open_loop:
            self.continue_plugin_state = Status.FAILURE
        elif god_map.tree.control_mode == god_map.tree.control_mode.close_loop:
            self.continue_plugin_state = Status.RUNNING
        else:
            logging.logerr(f'{god_map.tree.control_mode} is not supported')
            raise GiskardException()

        # True to print sensor data
        self.show_data = False
        self.cancel = True
        self.plugin_canceled = (False, 0)

        self.init_data = True

    @profile
    def setup(self, timeout):

        self.wrench_compensated_subscriber = rospy.Subscriber(self.wrench_topic, WrenchStamped,
                                                              self.get_rospy_data)
        return True

    @profile
    def get_rospy_data(self,
                       data_compensated: WrenchStamped):
        if self.init_data:
            self.init_data = False
            self.prev_values = [data_compensated] * (self.order + 1)
            self.whole_data = {'unfiltered': [data_compensated],
                               'filtered': [data_compensated]}

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

        self.filtered_data = {'stamp': filtered_data.header.stamp,
                              'seq': filtered_data.header.seq,
                              'x_force': filtered_data.wrench.force.x,
                              'y_force': filtered_data.wrench.force.y,
                              'z_force': filtered_data.wrench.force.z,
                              'x_torque': filtered_data.wrench.torque.x,
                              'y_torque': filtered_data.wrench.torque.y,
                              'z_torque': filtered_data.wrench.torque.z}

        self.filtered_derivatives = {'stamp': filtered_data.header.stamp,
                                     'seq': filtered_data.header.seq,
                                     'x_force': filtered_data.wrench.force.x - self.whole_data['filtered'][
                                         -1].wrench.force.x,
                                     'y_force': filtered_data.wrench.force.y - self.whole_data['filtered'][
                                         -1].wrench.force.y,
                                     'z_force': filtered_data.wrench.force.z - self.whole_data['filtered'][
                                         -1].wrench.force.z,
                                     'x_torque': filtered_data.wrench.torque.x - self.whole_data['filtered'][
                                         -1].wrench.torque.x,
                                     'y_torque': filtered_data.wrench.torque.y - self.whole_data['filtered'][
                                         -1].wrench.torque.y,
                                     'z_torque': filtered_data.wrench.torque.z - self.whole_data['filtered'][
                                         -1].wrench.torque.z}

        self.whole_data['unfiltered'].append(data_compensated)
        self.whole_data['filtered'].append(filtered_data)

    def cancel_goal_check(self):
        """
        place with frontal grasping: force: -x, torque: y
        """
        # FIXME: condition for norm (np.linag.norm)

        condition_triggered = self.condition(self.filtered_data, self.filtered_derivatives)

        if condition_triggered:
            self.plugin_canceled = (True, self.filtered_data['stamp'], self.filtered_data['seq'])

            if self.cancel:
                logging.loginfo(f'tree cycle:  {god_map.tree.count}')

                self.cancel_condition = True

    @catch_and_raise_to_blackboard
    @profile
    def update(self):

        self.save_data()

        if self.cancel_condition:
            logging.loginfo('goal canceled')

            raise GiskardException()

        return self.continue_plugin_state

    def save_data(self):

        types = ['filtered', 'unfiltered']
        keys = ['timestamp', 'seq', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']

        package_path = self.rospack.get_path('suturo_manipulation')
        standard_path = package_path + '/src/suturo_manipulation/'

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
