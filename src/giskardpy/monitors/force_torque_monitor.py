import string
from typing import Optional

import geometry_msgs
import rospy
from geometry_msgs.msg import WrenchStamped

import giskardpy.casadi_wrapper as cas
from giskardpy.god_map import god_map
from giskardpy.monitors.monitors import PayloadMonitor
from giskardpy.suturo_types import ForceTorqueThresholds, ObjectTypes
from giskardpy.utils import logging


class PayloadForceTorque(PayloadMonitor):
    """
    The Payload_Force class creates a monitor for the usage of the HSRs Force-Torque Sensor.
    This makes it possible for goals which use the Force-Torque Sensor to be used with Monitors,
    specifically to end/hold a goal automatically when a certain Force/Torque Threshold is being surpassed.
    """

    def __init__(self,
                 # is_raw is necessary to switch between the raw and the compensated topic
                 is_raw: bool,
                 # threshold_name is needed here for the class to be able to handle the suturo_types appropriately
                 threshold_name: string,
                 # object_type is needed to differentiate between objects with different placing thresholds
                 object_type: Optional[str] = None,
                 topic: string = "/hsrb/wrist_wrench/compensated",
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol
                 ):
        """
        :param is_raw: checks if the /hsrb/wrist_wrench/raw topic is being listened to
        :param threshold_name: contains the name of the threshold that will be used (normally an action e.g Placing)
        :param object_type: is used to determine the type of object that is being placed, is left empty if no object is being placed
        :param topic: the name of the topic
        :param name: name of the monitor class
        :param start_condition: the start condition of the monitor
        """
        if is_raw:
            logging.logwarn("ForceTorque Monitor is now listening to the /hsrb/wrist_wrench/raw Topic!")
            topic = "/hsrb/wrist_wrench/raw"

        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.object_type = object_type
        self.threshold_name = threshold_name
        self.topic = topic
        self.wrench = WrenchStamped()
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)

    def cb(self, data: WrenchStamped):
        self.wrench = data

    def force_T_map_transform(self, picker):
        """
        The force_T_map_transform method is used to transform the Vector data from the
        force-torque sensor frame into the map frame, so that the axis stay
        the same, to ensure that the threshold check is actually done on the relevant/correct axis.
        """
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

    # TODO: Add proper checks for all necessary Items!
    def __call__(self):

        rob_force = self.force_T_map_transform(1)
        rob_torque = self.force_T_map_transform(2)
        # TODO: Add more threshold cases and determine whether the normal grasp action is even needed currently
        if self.threshold_name == ForceTorqueThresholds.FT_GraspWithCare.value:

            if self.object_type == ObjectTypes.OT_Standard.value:

                force_threshold = 0.2
                torque_threshold = 0.02

                if (abs(rob_force.vector.y) > force_threshold or
                        abs(rob_torque.vector.y) > torque_threshold):
                    self.state = True
                    print(f'HIT GWC: {rob_force.vector.x};{rob_torque.vector.y}')
                else:
                    self.state = False
                    print(f'MISS GWC!: {rob_force.vector.x};{rob_torque.vector.y}')

            elif self.object_type == ObjectTypes.OT_Cutlery.value:

                #  TODO: Add proper grasping logic
                print("IT JUST WORKS - Todd Howard, at some point")

            elif self.object_type == ObjectTypes.OT_Plate.value:
                #  TODO: Add proper grasping logic
                print("IT JUST WORKS - Todd Howard, at some point")

            elif self.object_type == ObjectTypes.OT_Tray.value:
                #  TODO: Add proper grasping logic
                print("IT JUST WORKS - Todd Howard, at some point")

            else:
                logging.logerr("No valid object_type found, unable to determine placing thresholds!")

        # TODO: Add thresholds and cases for other object types
        elif self.threshold_name == ForceTorqueThresholds.FT_Placing.value:

            if self.object_type == ObjectTypes.OT_Standard.value:
                force_x_threshold = 2.34
                force_z_threshold = 1.0
                torque_y_threshold = 0.45

                if (abs(rob_force.vector.x) >= force_x_threshold and
                        abs(rob_force.vector.z) >= force_z_threshold and
                        abs(rob_torque.vector.y) >= torque_y_threshold):

                    self.state = True
                    print(f'HIT PLACING: {rob_force.vector.x};{rob_force.vector.z}')
                else:
                    self.state = False
                    print(f'MISS PLACING!: {rob_force.vector.x};{rob_force.vector.z}')

            elif self.object_type == ObjectTypes.OT_Cutlery.value:
                #  TODO: Add proper placing logic
                #  Check cutlery placing force-> might need raw topic too?
                print("IT JUST WORKS - Todd Howard, at some point")

            elif self.object_type == ObjectTypes.OT_Plate.value:
                #  TODO: Add proper placing logic
                print("IT JUST WORKS - Todd Howard, at some point")

            elif self.object_type == ObjectTypes.OT_Tray.value:
                #  TODO: Add proper placing logic
                print("IT JUST WORKS - Todd Howard, at some point")

            else:
                logging.logerr("No valid object_type found, unable to determine placing thresholds!")

        elif (self.threshold_name == ForceTorqueThresholds.FT_GraspCutlery.value
              & self.topic == "/hsrb/wrist_wrench/raw"):
            # TODO: Add proper Thresholds and Checks for grasping Cutlery
            force_x_threshold = 0
            force_y_threshold = 0
            force_z_threshold = 0

            if (abs(rob_force.vector.x) >= force_x_threshold and
                    abs(rob_force.vector.y) >= force_y_threshold and
                    abs(rob_force.vector.z) >= force_z_threshold):

                self.state = True

            else:
                self.state = False

        elif self.threshold_name == ForceTorqueThresholds.FT_Door.value:
            # TODO: Establish needed values and add logic for door handling
            force_x_threshold = 0
            force_y_threshold = 0
            force_z_threshold = 0

        elif self.threshold_name == ForceTorqueThresholds.FT_DishDoor.value:
            # TODO: Establish needed values and add logic for dishwasher door handling
            force_x_threshold = 0
            force_y_threshold = 0
            force_z_threshold = 0

        else:
            logging.logerr("No valid threshold_name found, unable to determine action!")
