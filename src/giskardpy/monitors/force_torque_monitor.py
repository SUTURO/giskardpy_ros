import os
import string
from copy import copy
from typing import Optional

import geometry_msgs
from rospy import wait_for_message

from giskardpy.utils import logging

if 'GITHUB_WORKFLOW' not in os.environ:
    pass
import rospy
from geometry_msgs.msg import WrenchStamped

import giskardpy.casadi_wrapper as cas
from giskardpy.god_map import god_map
from giskardpy.monitors.monitors import PayloadMonitor
from giskardpy.suturo_types import ForceTorqueThresholds, ObjectTypes


class PayloadForceTorque(PayloadMonitor):
    def __init__(self,
                 # threshold_name is needed here for the class to be able to handle the suturo_types appropriately
                 threshold_name: str,
                 topic: str,
                 # object_type is needed to differentiate between objects with different placing thresholds
                 object_type: str,
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 stay_true: bool = True):
        """
        The PayloadForceTorque class creates a monitor for the usage of the HSRs Force-Torque Sensor.
        This makes it possible for goals which use the Force-Torque Sensor to be used with Monitors,
        specifically to end/hold a goal automatically when a certain Force/Torque Threshold is being surpassed.

        :param threshold_name: contains the name of the threshold that will be used (normally an action e.g. Placing)
        :param object_type: is used to determine the type of object that is being placed, is left empty if no object is being placed
        :param topic: the name of the topic
        :param name: name of the monitor class
        :param start_condition: the start condition of the monitor
        """

        super().__init__(name=name, stay_true=stay_true, start_condition=start_condition, run_call_in_thread=False)
        self.object_type = object_type
        self.threshold_name = threshold_name
        self.topic = topic
        self.wrench = WrenchStamped()
        self.bf = god_map.world.search_for_link_name('base_footprint')
        self.sensor_frame = god_map.world.search_for_link_name(wait_for_message(topic, WrenchStamped).header.frame_id)
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)

    def cb(self, data: WrenchStamped):
        self.rob_force = self.force_T_map_transform(data, 1)
        self.rob_torque = self.force_T_map_transform(data, 2)

    def force_T_map_transform(self, wrench, picker):
        """
        The force_T_map_transform method is used to transform the Vector data from the
        force-torque sensor frame into the HSRs base frame, so that the axis stay
        the same, to ensure that the threshold check is actually done on the correct axis,
        since conversion into the map frame can lead to different values depending on how the map is recorded.
        """
        wrench.header.frame_id = self.sensor_frame

        vstampF = geometry_msgs.msg.Vector3Stamped(header=wrench.header, vector=wrench.wrench.force)
        vstampT = geometry_msgs.msg.Vector3Stamped(header=wrench.header, vector=wrench.wrench.torque)

        force_transformed = god_map.world.transform_vector(self.bf, vstampF)

        torque_transformed = god_map.world.transform_vector(self.bf, vstampT)

        # print("Force:", force_transformed.vector.x, force_transformed.vector.y, force_transformed.vector.z)
        # print("Torque:", torque_transformed.vector.x, torque_transformed.vector.y, torque_transformed.vector.z)

        if picker == 1:

            return force_transformed

        elif picker == 2:

            return torque_transformed

    # TODO: Add proper checks for all necessary Items!
    def __call__(self):
        rob_force = copy(self.rob_force)
        rob_torque = copy(self.rob_torque)

        if self.threshold_name == ForceTorqueThresholds.FT_GraspWithCare.value:

            # case for grasping "normal" objects (namely Milk, Cereal and cups)
            if self.object_type == ObjectTypes.OT_Standard.value:

                torque_threshold = 2

                if abs(rob_torque.vector.y) > torque_threshold:
                    self.state = True
                    logging.loginfo(f'HIT GWC: {rob_torque.vector.y}')
                else:
                    self.state = False

            # case for grasping cutlery
            elif self.object_type == ObjectTypes.OT_Cutlery.value:

                force_threshold = 85

                if abs(rob_force.vector.z) > force_threshold:
                    self.state = True
                    print(f'HIT GWC: {rob_force.vector.z};{rob_torque.vector.y}')
                else:
                    self.state = False
                    print(f'MISS GWC: {rob_force.vector.z};{rob_torque.vector.y}')

            # case for grasping plate
            # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
            elif self.object_type == ObjectTypes.OT_Plate.value:

                torque_threshold = 0.02

                if (abs(rob_force.vector.y) > torque_threshold or
                        abs(rob_torque.vector.y) > torque_threshold):
                    self.state = False
                    print(f'HIT GWC: {rob_force.vector.x};{rob_torque.vector.y}')
                else:
                    self.state = True
                    raise Exception("HSR failed to Grasp Object, Grasping threshold has been Undershot.")

            # case for grasping Bowl
            elif self.object_type == ObjectTypes.OT_Bowl.value:

                force_threshold = 50.0

                if abs(rob_force.vector.z) > force_threshold:
                    self.state = True
                    print(rob_force.vector.z)
                else:
                    self.state = False
                    print(f'MISS GWC:{rob_force.vector.z}')

            # if no valid object_type has been declared in method parameters
            else:
                raise Exception("No valid object_type found, unable to determine placing thresholds!")

        # TODO: Add thresholds and cases for other object types
        elif self.threshold_name == ForceTorqueThresholds.FT_Placing.value:

            # case for placing "normal" objects (namely Milk, Cereal and cups)
            if self.object_type == ObjectTypes.OT_Standard.value:

                force_z_threshold = 35

                if abs(rob_force.vector.z) >= force_z_threshold:

                    self.state = True
                    logging.loginfo(
                        f'HIT PLACING!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_force.vector.y}')
                else:
                    self.state = False

            # case for placing cutlery
            elif self.object_type == ObjectTypes.OT_Cutlery.value:

                if (self.threshold_name == ForceTorqueThresholds.FT_PlaceCutlery.value
                        & self.topic == "compensated/diff"):
                    self.topic = "filtered_raw"
                    # TODO: Add proper Thresholds and Checks for placing Cutlery
                    logging.loginfo(f'filtered Force: {rob_force}')
                    logging.loginfo(f'filtered Torque: {rob_torque}')

                    force_z_threshold = 35

                    if abs(rob_force.vector.z) >= force_z_threshold:

                        self.state = True
                        logging.loginfo(
                            f'HIT CUTLERY!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                    else:
                        self.state = False
                        logging.loginfo(
                            f'MISS CUTLERY!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')

            # case for placing plates
            # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
            elif self.object_type == ObjectTypes.OT_Plate.value:
                #  TODO: Add proper placing logic for Plate
                force_z_threshold = 1.0

                if abs(rob_force.vector.z) >= force_z_threshold:

                    self.state = True
                    print(f'HIT PLACING: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                else:
                    self.state = False
                    print(f'MISS PLACING!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')

            # case for placing bowls
            elif self.object_type == ObjectTypes.OT_Bowl.value:

                force_z_threshold = 35  # 1.0

                if abs(rob_force.vector.z) >= force_z_threshold:

                    self.state = True
                    print(f'HIT PLACING: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                else:
                    self.state = False
            # if no valid object_type has been declared in method parameters
            else:
                raise Exception("No valid object_type found, unable to determine placing thresholds!")

        else:
            raise Exception("No valid threshold_name found, unable to determine proper course of action!")
