import string
from typing import Optional
import geometry_msgs
from geometry_msgs.msg import WrenchStamped
import giskardpy.casadi_wrapper as cas
import numpy as np
import rospy

from giskardpy.monitors.payload_monitors import PayloadMonitor
from giskardpy.suturo_types import ForceTorqueThresholds


class Payload_Force(PayloadMonitor):
    """
    The Payload_Force class creates a monitor for the usage of the HSRs Force-Torque Sensor.
    This makes it possible for goals which use the Force-Torque Sensor to be used with Monitors,
    specifically to end/finish a goal automatically when a certain Force/Torque Threshold is being passed.
    """
    def __init__(self,
                 # use /hsrb/wrist_wrench/compensated for actual HSR, for testing feel free to change it
                 topic: string = "/hsrb/wrist_wrench/compensated",
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol):
        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.topic = topic
        self.wrench = WrenchStamped()
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)

    def cb(self, data: WrenchStamped):
        self.wrench = data

    def __call__(self, threshold_name: Optional[str] = None):

        if threshold_name == ForceTorqueThresholds.FT_GraspWithCare.value:
            force_threshold = 5.0

            if (abs(self.wrench.wrench.force.x) >= force_threshold or
                    abs(self.wrench.wrench.force.y) >= force_threshold or
                    abs(self.wrench.wrench.force.z) >= force_threshold):

                self.state = True

            else:
                self.state = False

        elif threshold_name == ForceTorqueThresholds.FT_Placing.value:
            force_x_threshold = 0.0
            force_z_threshold = 1.0
            torque_y_threshold = 0.15

            if abs(self.wrench.wrench.force.z) >= force_z_threshold:  # abs(self.wrench.wrench.torque.y) >= torque_y_threshold

                self.state = True

            elif (abs(self.wrench.wrench.force.x) <= force_x_threshold or
                  abs(self.wrench.wrench.torque.y) >= torque_y_threshold):

                self.state = True

            else:
                self.state = False
        # TODO: Make another conditional for door handling (might be included as part of GraspCarefully,
        #  in that case Rework that conditional)
        """ If conditional for initial testing purposes
        if abs(self.wrench.wrench.force.z) >= force_threshold or abs(self.wrench.wrench.torque.y) >= torque_threshold:
            self.state = True
            print(f' SUCCESS! F_z: {self.wrench.wrench.force.z}')
            print(f' SUCCESS! T_y: {self.wrench.wrench.torque.y}')
            print("---------------------------------------------")
        else:
            self.state = False
            print(f'F_z: {self.wrench.wrench.force.z}')
            print(f'T_y: {self.wrench.wrench.torque.y}')
            print("---------------------------------------------")
        """
