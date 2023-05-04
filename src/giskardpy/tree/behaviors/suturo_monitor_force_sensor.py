import rospy
from geometry_msgs.msg import WrenchStamped
from py_trees import Status

from giskardpy import identifier
from giskardpy.data_types import JointStates
from giskardpy.exceptions import MonitorForceException
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.tree.garden import TreeManager
from giskardpy.utils import logging
from giskardpy.utils.utils import catch_and_raise_to_blackboard

import numpy as np


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name):
        super().__init__(name)
        self.cancel_condition = False
        self.name = name
        self.wrench_compensated_subscriber = None

        self.force_threshold = -3.0
        self.torque_threshold = 0.5
        # self.force_derivative_threshold = 50
        # self.force_derivative_threshold = 50


        self.wrench_compensated_force_data_x = []
        self.wrench_compensated_force_data_y = []
        self.wrench_compensated_force_data_z = []
        self.wrench_compensated_torque_data_x = []
        self.wrench_compensated_torque_data_y = []
        self.wrench_compensated_torque_data_z = []
        self.wrench_compensated_latest_data = WrenchStamped()

        # True to print sensor data
        self.show_data = False

    @profile
    def setup(self, timeout):
        self.wrench_compensated_subscriber = rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped,
                                                              self.get_rospy_data)

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

        # force_average_x = np.mean(self.wrench_compensated_force_data_x)

        force_axis = self.wrench_compensated_latest_data.wrench.force.x
        torque_axis = self.wrench_compensated_latest_data.wrench.force.y

        force_cancel = force_axis < self.force_threshold
        torque_cancel = torque_axis > self.torque_threshold

        if force_cancel or torque_cancel:
            print(f'force cancel: {force_cancel}')
            print(f'torque cancel: {torque_cancel}')

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

    @catch_and_raise_to_blackboard
    @profile
    def update(self):

        if self.cancel_condition:

            tree = self.tree
            tree.remove_node(self.name)

            print('goal canceled')

            raise MonitorForceException

        return Status.SUCCESS
