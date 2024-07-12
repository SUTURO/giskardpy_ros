import rclpy
from rclpy.node import Node

node: Node = None


def init_node(node_name: str):
    global node
    if node is not None:
        raise Exception('ros node already initialized.')
    rclpy.init()
    node = Node(node_name)
