from typing import List

from py_trees.common import Status
from std_msgs.msg import Float64MultiArray

from giskardpy.data_types.data_types import PrefixName
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.utils.decorators import record_time
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard


class JointGroupVelController(GiskardBehavior):
    @profile
    def __init__(self, cmd_topic: str, joints: List[PrefixName]):
        super().__init__()
        self.cmd_topic = cmd_topic
        # self.cmd_topic = self.search_for_subscriber_with_type(Float64MultiArray)
        self.cmd_pub = rospy.node.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # self.joint_names = self.get_joints()
        self.joint_names = joints
        god_map.world.register_controlled_joints(self.joint_names)
        self.msg = None
        get_middleware().loginfo(f'Created publisher for {self.cmd_topic} for {self.joint_names}')

    @catch_and_raise_to_blackboard
    @record_time
    @profile
    def update(self):
        msg = Float64MultiArray()
        for i, joint_name in enumerate(self.joint_names):
            msg.data.append(god_map.world.state[joint_name].velocity)
        self.cmd_pub.publish(msg)
        return Status.RUNNING

    def terminate(self, new_status):
        msg = Float64MultiArray()
        for joint_name in self.joint_names:
            msg.data.append(0.0)
        self.cmd_pub.publish(msg)
        super().terminate(new_status)
