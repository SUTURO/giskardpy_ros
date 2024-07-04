import builtins

from rclpy.node import Node

import giskardpy
from giskardpy_ros.ros2.interface import ROS2Wrapper

try:
    builtins.profile  # type: ignore
except AttributeError:
    # No line profiler, provide a pass-through version
    def profile(func):
        return func


    builtins.profile = profile  # type: ignore

giskardpy.middleware.middleware = ROS2Wrapper()
ros_node: Node = giskardpy.middleware.middleware.node
