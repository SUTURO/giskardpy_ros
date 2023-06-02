import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal, JointTolerance
from geometry_msgs.msg import WrenchStamped
from giskard_msgs.msg import MoveResult
from py_trees import Status

from giskardpy import identifier
from giskardpy.data_types import JointStates
from giskardpy.exceptions import MonitorForceException
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils import logging
from giskardpy.utils.decorators import catch_and_raise_to_blackboard

import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

import numpy as np


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name, conditions):
        super().__init__(name)
        self.arm_trajectory_publisher = None
        self.cancel_condition = False
        self.name = name
        self.wrench_compensated_subscriber = None

        self.force_threshold = -1.0
        self.torque_threshold = 1.0
        # self.force_derivative_threshold = 50
        # self.force_derivative_threshold = 50

        self.wrench_compensated_force_data_x = []
        self.wrench_compensated_force_data_y = []
        self.wrench_compensated_force_data_z = []
        self.wrench_compensated_torque_data_x = []
        self.wrench_compensated_torque_data_y = []
        self.wrench_compensated_torque_data_z = []
        self.wrench_compensated_latest_data = WrenchStamped()

        self.conditions = conditions

        self.counter = 0

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

        # force_average_x = np.mean(self.wrench_compensated_force_data_x)

        force_axis = self.wrench_compensated_latest_data.wrench.force.x
        torque_axis = self.wrench_compensated_latest_data.wrench.force.y

        force_cancel = force_axis < self.force_threshold
        torque_cancel = torque_axis > self.torque_threshold

        if self.counter > 2:
            self.cancel_condition = True

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
            print('goal canceled')

            self.recover()

            #return Status.SUCCESS
            raise MonitorForceException

        self.counter += 1

        return Status.FAILURE

    def terminate(self, new_status):

        if self.cancel_condition:
            tree = self.tree
            tree.remove_node(self.name)

    def recover(self):

        joint_names = ['arm_lift_joint', 'arm_flex_joint',
                       'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

        joint_positions = [self.world.state.get(self.world.search_for_joint_name(joint_name)).position
                           for joint_name in joint_names]

        joint_positions[0] += 0.01

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
