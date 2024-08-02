import rclpy
from py_trees.common import Status
from rcl_interfaces.srv import GetParameterTypes, GetParameterTypes_Request, GetParameters_Response, \
    GetParameters_Request, GetParameters
from std_msgs.msg import Float64MultiArray

from giskardpy.data_types.data_types import KeyDefaultDict
from giskardpy.god_map import god_map
from giskardpy_ros.ros2.ros2_interface import search_for_subscriber_with_type

from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard


class JointGroupVelController(GiskardBehavior):
    @profile
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.node_name = node_name
        self.param_service = rospy.node.create_client(GetParameters,
                                                      f'{self.node_name}/get_parameters')
        self.cmd_topic = search_for_subscriber_with_type(self.node_name, Float64MultiArray)
        self.cmd_pub = rospy.node.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        self.joint_names = self.get_joints()
        for i in range(len(self.joint_names)):
            self.joint_names[i] = god_map.world.search_for_joint_name(self.joint_names[i])
        god_map.world.register_controlled_joints(self.joint_names)
        self.msg = None

    def get_joints(self):
        req = GetParameters_Request()
        req.names = ['joints']
        res: GetParameters_Response = self.param_service.call(req, timeout_sec=5)
        return res.values[0].string_array_value

    @profile
    def initialise(self):
        def f(joint_symbol):
            return god_map.expr_to_key[joint_symbol][-2]

        self.symbol_to_joint_map = KeyDefaultDict(f)
        super().initialise()

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
