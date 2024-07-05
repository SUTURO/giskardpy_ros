import os
os.environ['ROS_PYTHON_CHECK_FIELDS'] = '1'

import builtins
import signal
import sys
import traceback

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

def sigabrt_handler(signum, frame):
    print("SIGABRT received. Stack trace:")
    traceback.print_stack(frame)
    sys.exit(1)

# Set the signal handler for SIGABRT
signal.signal(signal.SIGABRT, sigabrt_handler)

giskardpy.middleware.middleware = ROS2Wrapper()
ros_node: Node = giskardpy.middleware.middleware.node
