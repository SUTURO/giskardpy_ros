import os
import string
from typing import Optional

import geometry_msgs

if 'GITHUB_WORKFLOW' not in os.environ:
    pass
import rospy
from geometry_msgs.msg import WrenchStamped

import giskardpy.casadi_wrapper as cas
from giskardpy.god_map import god_map
from giskardpy.monitors.monitors import PayloadMonitor
from giskardpy.suturo_types import ForceTorqueThresholds, ObjectTypes


class PayloadForceTorque(PayloadMonitor):
    """
    The PayloadForceTorque class creates a monitor for the usage of the HSRs Force-Torque Sensor.
    This makes it possible for goals which use the Force-Torque Sensor to be used with Monitors,
    specifically to end/hold a goal automatically when a certain Force/Torque Threshold is being surpassed.
    """

    def __init__(self,
                 # threshold_name is needed here for the class to be able to handle the suturo_types appropriately
                 threshold_name: string,
                 # object_type is needed to differentiate between objects with different placing thresholds
                 object_type: Optional[str] = None,
                 topic: string = "/compensated/diff",
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol
                 ):
        """
        :param threshold_name: contains the name of the threshold that will be used (normally an action e.g. Placing)
        :param object_type: is used to determine the type of object that is being placed, is left empty if no object is being placed
        :param topic: the name of the topic
        :param name: name of the monitor class
        :param start_condition: the start condition of the monitor
        """

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

        if self.threshold_name == ForceTorqueThresholds.FT_GraspWithCare.value:

            # Basic idea for checking whether an object has successfully been grasped or not:
            # return false as long as threshold is being surpassed;
            # return true when it stops being surpassed, so that goal stops when object is unexpectedly dropped
            # or the HSR failed to grasp them at all

            # case for grasping "normal" objects (namely Milk, Cereal and cups)
            if self.object_type == ObjectTypes.OT_Standard.value:

                force_threshold = 10.0

                if abs(rob_force.vector.y) > force_threshold:
                    self.state = True
                    print(f'HIT GWC: {rob_force.vector.y}')
                else:
                    self.state = False
                    print(f'MISS GWC: {rob_force.vector.y}')

            # case for grasping cutlery
            elif self.object_type == ObjectTypes.OT_Cutlery.value:
                self.topic = "filtered_raw/diff"
                # switch to filtered_raw / filtered_raw/diff
                force_threshold = -98.0
                torque_threshold = -3.8

                if (abs(rob_force.vector.z) > force_threshold or
                        abs(rob_torque.vector.y) > torque_threshold):
                    self.state = True
                    print(f'HIT GWC: {rob_force.vector.z};{rob_torque.vector.y}')
                else:
                    self.state = False
                    print(f'MISS GWC: {rob_force.vector.z};{rob_torque.vector.y}')

            # case for grasping plate
            # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
            elif self.object_type == ObjectTypes.OT_Plate.value:

                force_threshold = 0.2
                torque_threshold = 0.02

                if (abs(rob_force.vector.y) > force_threshold or
                        abs(rob_torque.vector.y) > torque_threshold):
                    self.state = False
                    print(f'HIT GWC: {rob_force.vector.x};{rob_torque.vector.y}')
                else:
                    self.state = True
                    raise Exception("HSR failed to Grasp Object, Grasping threshold has been Undershot.")

            # case for grasping Bowl
            elif self.object_type == ObjectTypes.OT_Bowl.value:

                force_threshold = 60.0

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
                force_x_threshold = 2.34
                force_z_threshold = 1.0
                torque_y_threshold = 0.45

                if (abs(rob_force.vector.x) >= force_x_threshold and
                        abs(rob_force.vector.z) >= force_z_threshold and
                        abs(rob_torque.vector.y) >= torque_y_threshold):

                    self.state = True
                    print(f'HIT PLACING!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                else:
                    self.state = False
                    print(f'MISS PLACING!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')

            # case for placing cutlery
            elif self.object_type == ObjectTypes.OT_Cutlery.value:

                if (self.threshold_name == ForceTorqueThresholds.FT_PlaceCutlery.value
                        & self.topic == "compensated/diff"):
                    self.topic = "filtered_raw"
                    # TODO: Add proper Thresholds and Checks for placing Cutlery
                    print(f'filtered Force: {rob_force}')
                    print(f'filtered Torque: {rob_torque}')
                    force_x_threshold = 0
                    force_y_threshold = 0
                    force_z_threshold = 0

                    if (abs(rob_force.vector.x) >= force_x_threshold and
                            abs(rob_force.vector.y) >= force_y_threshold and
                            abs(rob_force.vector.z) >= force_z_threshold):

                        self.state = True
                        print(f'HIT CUTLERY!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                    else:
                        self.state = False
                        print(f'MISS CUTLERY!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')

            # case for placing plates
            # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
            elif self.object_type == ObjectTypes.OT_Plate.value:
                #  TODO: Add proper placing logic for Plate
                force_x_threshold = 2.34
                force_z_threshold = 1.0
                torque_y_threshold = 0.45

                if (abs(rob_force.vector.x) >= force_x_threshold and
                        abs(rob_force.vector.z) >= force_z_threshold and
                        abs(rob_torque.vector.y) >= torque_y_threshold):

                    self.state = True
                    print(f'HIT PLACING: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                else:
                    self.state = False
                    print(f'MISS PLACING!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')

            # case for placing bowls
            elif self.object_type == ObjectTypes.OT_Bowl.value:

                force_x_threshold = 3.24
                force_z_threshold = 3.0
                torque_y_threshold = 3.00

                if (abs(rob_force.vector.x) >= force_x_threshold or
                        abs(rob_force.vector.z) >= force_z_threshold or
                        abs(rob_torque.vector.y) >= torque_y_threshold):

                    self.state = True
                    print(f'HIT PLACING: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
                else:
                    self.state = False
                    #print(f'MISS PLACING!: X:{rob_force.vector.x};Z:{rob_force.vector.z};Y:{rob_torque.vector.y}')
            # if no valid object_type has been declared in method parameters
            else:
                raise Exception("No valid object_type found, unable to determine placing thresholds!")

        else:
            raise Exception("No valid threshold_name found, unable to determine proper course of action!")
