from typing import Optional

import controller_manager as cm
import numpy as np
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers_Response
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.world_config import WorldWithOmniDriveRobot, WorldConfig
from giskardpy_ros.configs.giskard import RobotInterfaceConfig
from giskardpy.data_types.data_types import Derivatives, PrefixName
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy_ros.ros2 import ros2_interface, rospy
import giskardpy_ros.ros2.tfwrapper as tf
from giskardpy_ros.ros2.msg_converter import msg_type_as_str


class GenericWorldConfig(WorldConfig):
    robot_name = ''

    def setup(self):
        self.urdf = ros2_interface.get_robot_description()
        self.map_name = PrefixName(tf.get_tf_root())
        with self.world.modify_world():
            self.set_default_limits({Derivatives.velocity: 0.2,
                                     Derivatives.acceleration: np.inf,
                                     Derivatives.jerk: 30})
            self.add_robot_urdf(self.urdf, self.robot_name)
            root_link_name = self.get_root_link_of_group(self.robot_group_name)
            if root_link_name.short_name != self.map_name.short_name:
                self.add_empty_link(self.map_name)
                self.add_fixed_joint(parent_link=self.map_name, child_link=root_link_name)
            else:
                self.map_name = root_link_name


class GenericRobotInterface(RobotInterfaceConfig):
    drive_joint_name: str

    def __init__(self, controller_manager_name: str = 'controller_manager'):
        self.controller_manager_name = controller_manager_name

    def setup(self):
        controllers: ListControllers_Response = cm.list_controllers(node=rospy.node,
                                                                    controller_manager_name=self.controller_manager_name)
        controller: ControllerState
        for controller in controllers.controller:
            if controller.state == 'active':
                if controller.type == 'joint_state_broadcaster/JointStateBroadcaster':
                    node_name = controller.name
                    topics = rospy.node.get_publisher_names_and_types_by_node(node_name, '/')
                    for topic_name, topic_types in topics:
                        if topic_types[0] == msg_type_as_str(JointState):
                            self.sync_joint_state_topic(topic_name)
                            break
                elif controller.type == 'velocity_controllers/JointGroupVelocityController':
                    self.add_joint_velocity_group_controller(controller.name)


class GenericCollisionAvoidance(CollisionAvoidanceConfig):
    def __init__(self, drive_joint_name: str = 'brumbrum',
                 collision_checker: CollisionCheckerLib = CollisionCheckerLib.bpb):
        super().__init__(collision_checker=collision_checker)
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.load_self_collision_matrix('self_collision_matrices/iai/pr2.srdf')
        self.set_default_external_collision_avoidance(soft_threshold=0.1,
                                                      hard_threshold=0.0)
        for joint_name in ['r_wrist_roll_joint', 'l_wrist_roll_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        number_of_repeller=4,
                                                        soft_threshold=0.05,
                                                        hard_threshold=0.0,
                                                        max_velocity=0.2)
        for joint_name in ['r_wrist_flex_joint', 'l_wrist_flex_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        number_of_repeller=2,
                                                        soft_threshold=0.05,
                                                        hard_threshold=0.0,
                                                        max_velocity=0.2)
        for joint_name in ['r_elbow_flex_joint', 'l_elbow_flex_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        soft_threshold=0.05,
                                                        hard_threshold=0.0)
        for joint_name in ['r_forearm_roll_joint', 'l_forearm_roll_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        soft_threshold=0.025,
                                                        hard_threshold=0.0)
        self.fix_joints_for_collision_avoidance([
            'r_gripper_l_finger_joint',
            'l_gripper_l_finger_joint'
        ])
        self.overwrite_external_collision_avoidance(self.drive_joint_name,
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)
