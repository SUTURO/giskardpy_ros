import string
from typing import Optional
import geometry_msgs
from geometry_msgs.msg import WrenchStamped
import giskardpy.casadi_wrapper as cas
import numpy as np
import rospy

from giskardpy.monitors.payload_monitors import PayloadMonitor


class Payload_Force(PayloadMonitor):

    def __init__(self,
                 topic: string = "/hsrb/wrist_wrench/compensated",  # use /hsrb/wrist_wrench/compensated for actual robot
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol):
        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.topic = topic
        self.wrench = WrenchStamped()
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)

    def cb(self, data: WrenchStamped):
        self.wrench = data

    def __call__(self):

        # TODO: change Thresholds to assignable values depending on task and add more conditionals for more situations
        force_threshold = 8
        torque_threshold = 0.15

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
