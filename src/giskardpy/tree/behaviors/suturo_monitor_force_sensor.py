import rospy
from geometry_msgs.msg import WrenchStamped
from py_trees import Status

from giskardpy.data_types import JointStates
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils import logging
from giskardpy.utils.utils import catch_and_raise_to_blackboard


class MonitorForceSensor(GiskardBehavior):

    @profile
    def __init__(self, name):
        super().__init__(str(self))
        self.name = name
        self.wrench_compensated_subscriber = None

        self.wrench_compensated_force_data = []
        self.wrench_compensated_torque_data = []
        self.wrench_compensated_latest_data = WrenchStamped()

        # True to print sensor data
        self.show_data = False

    def setup(self, timeout):
        self.wrench_compensated_subscriber = rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped, self.show_msg)

        return True

    @catch_and_raise_to_blackboard
    @profile
    def update(self):
        return Status.SUCCESS

    def show_msg(self,
                 data_compensated: WrenchStamped):
        self.wrench_compensated_force_data.append(data_compensated.wrench.force)
        self.wrench_compensated_torque_data.append(data_compensated.wrench.torque)
        self.wrench_compensated_latest_data = data_compensated

        if self.show_data:
            print(data_compensated)


