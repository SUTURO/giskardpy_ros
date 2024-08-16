from threading import Thread

import rclpy
from ament_index_python import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from giskardpy.middleware import MiddlewareWrapper, set_middleware, get_middleware

node: Node = None
executor: MultiThreadedExecutor = None
spinner_thread: Thread = None


class ROS2Wrapper(MiddlewareWrapper):

    def loginfo(self, msg: str):
        global node
        node.get_logger().info(msg)

    def logwarn(self, msg: str):
        global node
        node.get_logger().warn(msg)

    def logerr(self, msg: str):
        global node
        node.get_logger().error(msg)

    def logdebug(self, msg: str):
        global node
        node.get_logger().debug(msg)

    def logfatal(self, msg: str):
        global node
        node.get_logger().fatal(msg)

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
            return path.replace('file://', '')


def heart():
    global node, executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.get_logger().info('Giskard died.')


def init_node(node_name: str) -> None:
    global node, spinner_thread, executor
    if node is not None:
        get_middleware().logwarn('ros node already initialized.')
        return
    rclpy.init()
    node = Node(node_name)
    spinner_thread = Thread(target=heart, daemon=True)
    set_middleware(ROS2Wrapper())
    spinner_thread.start()
