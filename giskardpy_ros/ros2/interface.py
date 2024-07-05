import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node

from giskardpy.middleware import MiddlewareWrapper


class ROS2Wrapper(MiddlewareWrapper):
    node: Node

    def __init__(self) -> None:
        super().__init__()
        rclpy.init()
        self.node = Node('giskard')

    def loginfo(self, msg: str):
        self.node.get_logger().info(msg)

    def logwarn(self, msg: str):
        self.node.get_logger().warn(msg)

    def logerr(self, msg: str):
        self.node.get_logger().error(msg)

    def logdebug(self, msg: str):
        self.node.get_logger().debug(msg)

    def logfatal(self, msg: str):
        self.node.get_logger().fatal(msg)

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
                real_path = get_package_share_directory(package_name)
                result += f'{real_path}/{suffix}'
            return result
        else:
            return path
