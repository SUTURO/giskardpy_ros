import string
from typing import Optional

import geometry_msgs
import rospy
from geometry_msgs.msg import WrenchStamped

import giskardpy.casadi_wrapper as cas
from giskardpy.god_map import god_map
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
                 # threshold_name is needed here for the class to be able to handle the suturo_types appropriately
                 threshold_name: Optional[str] = None):
        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.threshold_name = threshold_name
        self.topic = topic
        self.wrench = WrenchStamped()
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)

    def cb(self, data: WrenchStamped):
        self.wrench = data

    def force_T_map_transform(self, picker):
        self.wrench.header.frame_id = god_map.world.search_for_link_name(self.wrench.header.frame_id)

        vstampF = geometry_msgs.msg.Vector3Stamped(header=self.wrench.header, vector=self.wrench.wrench.force)
        vstampT = geometry_msgs.msg.Vector3Stamped(header=self.wrench.header, vector=self.wrench.wrench.torque)

        force_transformed = god_map.world.transform_vector('map', vstampF)

        torque_transformed = god_map.world.transform_vector('map', vstampT)

        # print("Force:", force_transformed.vector.x, force_transformed.vector.y, force_transformed.vector.z)
        # print("Torque:", torque_transformed.vector.x, torque_transformed.vector.y, torque_transformed.vector.z)

        if picker == 1:

            return force_transformed

        elif picker == 2:

            return torque_transformed

    def __call__(self):

        rob_force = self.force_T_map_transform(1)
        rob_torque = self.force_T_map_transform(2)

        if self.threshold_name == ForceTorqueThresholds.FT_GraspWithCare.value:

            force_threshold = 0.2  # might be y value above 0 (maybe torque above Zero too?)
            torque_threshold = 0.02
            # if (abs(rob_force.vector.x) >= force_threshold or
            #         abs(rob_force.vector.y) >= force_threshold or
            #         abs(rob_force.vector.z) >= force_threshold):

            if (abs(rob_force.vector.y) > force_threshold or
                    abs(rob_torque.vector.y) > torque_threshold):
                self.state = True
                print(f'HIT GWC: {rob_force.vector.x};{rob_torque.vector.y}')
            else:
                self.state = False
                print(f'MISS GWC!: {rob_force.vector.x};{rob_torque.vector.y}')
        elif self.threshold_name == ForceTorqueThresholds.FT_Placing.value:

            force_x_threshold = 10.0
            # force_z_threshold = 2.0  # placing is most likely Z (could be negative x value too)
            torque_y_threshold = 4  # might be negative y torque value (should still be in 0.x value area)

            if (abs(rob_force.vector.x) >= force_x_threshold or
                    abs(rob_torque.vector.y) > torque_y_threshold):

                self.state = True
                print(f'HIT PLACING1: {rob_force.vector.x};{rob_torque.vector.y}')

            else:
                self.state = False
                print(f'MISS PLACING!: {rob_force.vector.x};{rob_torque.vector.y}')
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
