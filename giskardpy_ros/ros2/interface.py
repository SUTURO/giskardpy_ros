from inspect import currentframe, getframeinfo
from typing import Optional, List

import rclpy
import rospkg
from rclpy.node import Node

from giskardpy.middleware import MiddlewareWrapper

rospack = rospkg.RosPack()


@profile
def generate_msg(msg):
    node_name = 'todo'
    new_msg = '[{}]: {}'.format(node_name, msg)
    if node_name == '/unnamed':
        print(new_msg)
    return new_msg


class ROS2Wrapper(MiddlewareWrapper):
    node: Node

    def __init__(self) -> None:
        super().__init__()
        rclpy.init()
        self.node = Node('giskard')

    def loginfo(self, msg: str):
        final_msg = generate_msg(msg)
        self.node.get_logger().loginfo(final_msg)

    def logwarn(self, msg: str):
        final_msg = generate_msg(msg)
        self.node.get_logger().logwarn(final_msg)

    def logerr(self, msg: str):
        final_msg = generate_msg(msg)
        self.node.get_logger().logerr(final_msg)

    def logdebug(self, msg: str):
        final_msg = generate_msg(msg)
        self.node.get_logger().logdebug(final_msg)

    def logfatal(self, msg: str):
        final_msg = generate_msg(msg)
        self.node.get_logger().logfatal(final_msg)

    def resolve_iri(cls, path: str) -> str:
        """
        e.g. 'package://giskardpy/data'
        """
        if 'package://' in path:
            split = path.split('package://')
            prefix = split[0]
            result = prefix
            for suffix in split[1:]:
                package_name, suffix = suffix.split('/', 1)
                real_path = rospack.get_path(package_name)
                result += f'{real_path}/{suffix}'
            return result
        else:
            return path
