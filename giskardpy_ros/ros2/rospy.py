from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

node: Node = None
executor: MultiThreadedExecutor = None
spinner_thread: Thread = None


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


def init_node(node_name: str):
    global node, spinner_thread, executor
    if node is not None:
        raise Exception('ros node already initialized.')
    rclpy.init()
    node = Node(node_name)
    spinner_thread = Thread(target=heart)
    spinner_thread.start()
