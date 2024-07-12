from py_trees.common import Status
from rclpy.duration import Duration

from giskardpy.god_map import god_map
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior


class SetTrackingStartTime(GiskardBehavior):
    def __init__(self, name, offset: float = 0.5):
        super().__init__(name)
        self.offset = Duration(seconds=offset)

    @profile
    def initialise(self):
        super().initialise()
        god_map.motion_start_time = (rospy.node.get_clock().now() + self.offset).nanoseconds / 1e9

    @profile
    def update(self):
        return Status.SUCCESS
