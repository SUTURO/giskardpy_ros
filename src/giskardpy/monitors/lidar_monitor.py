import string
from typing import Optional

import rospy
from sensor_msgs.msg import LaserScan

import giskardpy.casadi_wrapper as cas
from giskardpy.monitors.payload_monitors import PayloadMonitor


class LidarPayloadMonitor(PayloadMonitor):

    def __init__(self,
                 topic: string = 'hsrb/base_scan',
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol):
        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.topic = topic
        self.data = LaserScan()
        self.subscriber = rospy.Subscriber(name=topic, data_class=LaserScan, callback=self.lidar_cb)

    def lidar_cb(self, data: LaserScan):
        self.data = data

    def __call__(self):
        has_collision = False
        min_distance = 1  # Set your desired minimum distance in meters

        for i, range_value in enumerate(self.data.ranges):
            if self.data.range_min < range_value < self.data.range_max:
                if range_value < min_distance:
                    has_collision = True

        print(min(self.data.ranges))

        # print(f'{has_collision}, {min_distance}')
        self.state = has_collision
