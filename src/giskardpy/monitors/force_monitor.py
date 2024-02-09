import string
from typing import Optional
import geometry_msgs
from geometry_msgs.msg import WrenchStamped
import giskardpy.casadi_wrapper as cas
import numpy as np
import rospy
import giskardpy.utils.tfwrapper

from giskardpy.monitors.payload_monitors import PayloadMonitor
from giskardpy.suturo_types import ForceTorqueThresholds


class Payload_Force(PayloadMonitor):
    """
    The Payload_Force class creates a monitor for the usage of the HSRs Force-Torque Sensor.
    This makes it possible for goals which use the Force-Torque Sensor to be used with Monitors,
    specifically to end/hold a goal automatically when a certain Force/Torque Threshold is being passed.
    """

    def __init__(self,
                 # use /hsrb/wrist_wrench/compensated for actual HSR, for testing feel free to change
                 topic: string = "/hsrb/wrist_wrench/compensated",
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 threshold_name: Optional[str] = None):
        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.threshold_name = threshold_name
        self.topic = topic
        self.wrench = WrenchStamped()
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)

    def cb(self, data: WrenchStamped):
        self.wrench = data

    def force_T_map_transform(self):

        vstampF = geometry_msgs.msg.Vector3Stamped(header=self.wrench.header.frame_id, vector=self.wrench.wrench.force)
        vstampT = geometry_msgs.msg.Vector3Stamped(header=self.wrench.header.frame_id, vector=self.wrench.wrench.torque)

        force_transformed = giskardpy.utils.tfwrapper.transform_vector(target_frame='map',
                                                                       vector=vstampF,
                                                                       timeout=5)

        torque_transformed = giskardpy.utils.tfwrapper.transform_vector(target_frame='map',
                                                                        vector=vstampT,
                                                                        timeout=5)

    def __call__(self):

        if self.threshold_name == ForceTorqueThresholds.FT_GraspWithCare.value:
            force_threshold = 5

            if (abs(self.wrench.wrench.force.x) >= force_threshold or
                    abs(self.wrench.wrench.force.y) >= force_threshold or
                    abs(self.wrench.wrench.force.z) >= force_threshold):

                self.state = True
                print(
                    f'HIT GWC: {self.wrench.wrench.force.x};{self.wrench.wrench.force.y};{self.wrench.wrench.force.z}')
            else:
                self.state = False
                print(f'MISS GWC!')
        elif self.threshold_name == ForceTorqueThresholds.FT_Placing.value:
            force_x_threshold = 0.0
            force_z_threshold = 1.0
            torque_y_threshold = 0.15

            if abs(self.wrench.wrench.force.z) >= force_z_threshold:  # abs(self.wrench.wrench.torque.y) >= torque_y_threshold

                self.state = True
                print(f'HIT PLACING1: {self.wrench.wrench.force.z}')
            elif (abs(self.wrench.wrench.force.x) <= force_x_threshold or
                  abs(self.wrench.wrench.torque.y) >= torque_y_threshold):

                self.state = True
                print(f'HIT PLACING2: {self.wrench.wrench.force.x};{self.wrench.wrench.torque.y}')
            else:
                self.state = False
                print(f'MISS PLACING!')
        # TODO: Make another conditional for door handling (might be included as part of GraspCarefully,
        #  in that case Rework that conditional to handle multiple cases)
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
