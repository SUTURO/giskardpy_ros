from typing import Optional

from nav_msgs.msg import Odometry
from py_trees.common import Status

from giskardpy.data_types.data_types import PrefixName
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy_ros.ros2 import rospy, msg_converter
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard
from giskardpy.utils.decorators import record_time


class SyncOdometry(GiskardBehavior):

    @profile
    def __init__(self, odometry_topic: str, joint_name: Optional[PrefixName] = None, name_suffix: str = ''):
        self.odometry_topic = odometry_topic
        if not self.odometry_topic.startswith('/'):
            self.odometry_topic = '/' + self.odometry_topic
        super().__init__(str(self) + name_suffix)
        self.joint = god_map.world.get_drive_joint(joint_name=joint_name)
        self.odometry_sub = rospy.node.create_subscription(Odometry, self.odometry_topic, self.cb, 1)
        get_middleware().loginfo(f'Subscribed to {self.odometry_topic}')

    def __str__(self):
        return f'{super().__str__()} ({self.odometry_topic})'

    def cb(self, data: Odometry):
        self.odom = data

    @catch_and_raise_to_blackboard
    @record_time
    @profile
    def update(self):
        trans_matrix = msg_converter.ros_msg_to_giskard_obj(self.odom.pose.pose, god_map.world)
        self.joint.update_transform(trans_matrix)
        return Status.SUCCESS
