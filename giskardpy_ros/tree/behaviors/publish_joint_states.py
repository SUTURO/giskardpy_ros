from typing import Optional

from sensor_msgs.msg import JointState
from py_trees.common import Status

from giskardpy.god_map import god_map
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from line_profiler import profile


class PublishJointState(GiskardBehavior):
    @profile
    def __init__(self, name: Optional[str] = None, topic_name: Optional[str] = None, include_prefix: bool = False,
                 only_prismatic_and_revolute: bool = True):
        if name is None:
            name = self.__class__.__name__
        if topic_name is None:
            topic_name = '/joint_states'
        super().__init__(name)
        self.include_prefix = include_prefix
        self.cmd_topic = topic_name
        self.cmd_pub = rospy.node.create_publisher(JointState, self.cmd_topic, 10)
        if only_prismatic_and_revolute:
            self.joint_names = [k for k in god_map.world.joint_names if god_map.world.is_joint_revolute(k)
                                or god_map.world.is_joint_prismatic(k)]
        else:
            self.joint_names = list(god_map.world.state.keys())

    def update(self):
        msg = JointState()
        for joint_name in self.joint_names:
            if self.include_prefix:
                msg.name.append(joint_name.long_name)
            else:
                msg.name.append(joint_name.short_name)
            msg.position.append(god_map.world.state[joint_name].position)
            msg.velocity.append(god_map.world.state[joint_name].velocity)
        msg.header.stamp = rospy.node.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)
        return Status.SUCCESS
