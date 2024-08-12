from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, List, Dict

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers_Response
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from giskardpy.god_map import god_map
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.ros2.msg_converter import msg_type_as_str
from giskardpy_ros.ros2.ros2_interface import search_for_subscriber_with_type, get_parameters, \
    search_for_publisher_with_type
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from giskardpy_ros.tree.branches.giskard_bt import GiskardBT
from giskardpy_ros.tree.control_modes import ControlModes
from giskardpy.data_types.exceptions import SetupException
from giskardpy.model.world import WorldTree
from giskardpy.data_types.data_types import PrefixName, Derivatives
import controller_manager as cm


class RobotInterfaceConfig(ABC):
    def set_defaults(self):
        pass

    @abstractmethod
    def setup(self):
        """
        Implement this method to configure how Giskard can talk to the robot using it's self. methods.
        """

    @property
    def world(self) -> WorldTree:
        return god_map.world

    @property
    def robot_group_name(self) -> str:
        return self.world.robot_name

    def get_root_link_of_group(self, group_name: str) -> PrefixName:
        return self.world.groups[group_name].root_link_name

    @property
    def tree(self) -> GiskardBT:
        return GiskardBlackboard().tree

    @property
    def control_mode(self) -> ControlModes:
        return GiskardBlackboard().tree.control_mode

    def sync_odometry_topic(self, odometry_topic: str, joint_name: str):
        """
        Tell Giskard to sync an odometry joint added during by the world config.
        """
        joint_name = self.world.search_for_joint_name(joint_name)
        self.tree.wait_for_goal.synchronization.sync_odometry_topic(odometry_topic, joint_name)
        if GiskardBlackboard().tree.is_closed_loop():
            self.tree.control_loop_branch.closed_loop_synchronization.sync_odometry_topic_no_lock(
                odometry_topic,
                joint_name)

    def sync_6dof_joint_with_tf_frame(self, joint_name: str, tf_parent_frame: str, tf_child_frame: str):
        """
        Tell Giskard to sync a 6dof joint with a tf frame.
        """
        joint_name = self.world.search_for_joint_name(joint_name)
        self.tree.wait_for_goal.synchronization.sync_6dof_joint_with_tf_frame(joint_name,
                                                                              tf_parent_frame,
                                                                              tf_child_frame)
        if GiskardBlackboard().tree.is_closed_loop():
            self.tree.control_loop_branch.closed_loop_synchronization.sync_6dof_joint_with_tf_frame(
                joint_name,
                tf_parent_frame,
                tf_child_frame)

    def sync_joint_state_topic(self, topic_name: str, group_name: Optional[str] = None):
        """
        Tell Giskard to sync the world state with a joint state topic
        """
        if group_name is None:
            group_name = self.world.robot_name
        self.tree.wait_for_goal.synchronization.sync_joint_state_topic(group_name=group_name,
                                                                       topic_name=topic_name)
        if GiskardBlackboard().tree.is_closed_loop():
            self.tree.control_loop_branch.closed_loop_synchronization.sync_joint_state2_topic(
                group_name=group_name,
                topic_name=topic_name)

    def add_base_cmd_velocity(self,
                              cmd_vel_topic: str,
                              track_only_velocity: bool = False):
        """
        Tell Giskard how it can control an odom joint of the robot.
        :param cmd_vel_topic: a Twist topic
        :param track_only_velocity: The tracking mode. If true, any position error is not considered which makes
                                    the tracking smoother but less accurate.
        :param joint_name: name of the omni or diff drive joint. Doesn't need to be specified if there is only one.
        """
        # joint_name = self.world.search_for_joint_name(joint_name)
        if GiskardBlackboard().tree.is_closed_loop():
            self.tree.control_loop_branch.send_controls.add_send_cmd_velocity(cmd_vel_topic=cmd_vel_topic)
        elif GiskardBlackboard().tree.is_open_loop():
            self.tree.execute_traj.add_base_traj_action_server(cmd_vel_topic=cmd_vel_topic)

    def register_controlled_joints(self, joint_names: List[str], group_name: Optional[str] = None):
        """
        Tell Giskard which joints can be controlled. Giskard can usually figure this out on its own.
        Only used in standalone mode.
        :param group_name: Only needs to be specified, if there are more than two robots.
        """
        if self.control_mode != ControlModes.standalone:
            raise SetupException(f'Joints only need to be registered in {ControlModes.standalone.name} mode.')
        joint_names = [self.world.search_for_joint_name(j, group_name) for j in joint_names]
        self.world.register_controlled_joints(joint_names)

    def add_follow_joint_trajectory_server(self,
                                           namespace: str,
                                           group_name: Optional[str] = None,
                                           fill_velocity_values: bool = False,
                                           path_tolerance: Dict[Derivatives, float] = None):
        """
        Connect Giskard to a follow joint trajectory server. It will automatically figure out which joints are offered
        and can be controlled.
        :param namespace: namespace of the action server
        :param group_name: set if there are multiple robots
        :param fill_velocity_values: whether to fill the velocity entries in the message send to the robot
        """
        if group_name is None:
            group_name = self.world.robot_name
        if not GiskardBlackboard().tree.is_open_loop():
            raise SetupException('add_follow_joint_trajectory_server only works in planning mode')
        self.tree.execute_traj.add_follow_joint_traj_action_server(namespace=namespace,
                                                                   group_name=group_name,
                                                                   fill_velocity_values=fill_velocity_values,
                                                                   path_tolerance=path_tolerance)

    def discover_interfaces_from_controller_manager(self,
                                                    controller_manager_name: str = 'controller_manager',
                                                    whitelist: Optional[List[str]] = None) -> None:
        """
        :param whitelist: list all controllers that should get added, if None, giskard will search automatically
        """
        controllers: ListControllers_Response = cm.list_controllers(node=rospy.node,
                                                                    controller_manager_name=controller_manager_name)

        controllers_to_add = self.__filter_controllers_with_whitelist(controllers.controller, whitelist)

        for controller in controllers_to_add:
            if controller.state == 'active':
                if controller.type == 'joint_state_broadcaster/JointStateBroadcaster':
                    topic_name = search_for_publisher_with_type(topic_type=JointState,
                                                                node_name=controller.name)
                    self.sync_joint_state_topic(topic_name)
                elif controller.type == 'velocity_controllers/JointGroupVelocityController':
                    cmt_topic = search_for_subscriber_with_type(topic_type=Float64MultiArray,
                                                                node_name=controller.name)
                    joints = get_parameters(parameters=['joints'],
                                            node_name=controller.name).values[0].string_array_value
                    self.add_joint_velocity_group_controller(cmd_topic=cmt_topic, joints=joints)
                elif controller.type == 'diff_drive_controller/DiffDriveController':
                    self.add_base_cmd_velocity(controller.name)

    def __filter_controllers_with_whitelist(self, controllers: List[ControllerState], whitelist: Optional[List[str]]) \
            -> List[ControllerState]:
        controllers_to_add: List[ControllerState]
        if whitelist is None:
            return controllers
        else:
            available_controllers = {controller.name for controller in controllers}
            missing_controllers = [controller for controller in whitelist if controller not in available_controllers]
            if missing_controllers:
                raise ValueError(
                    f"The following controllers from the whitelist are not available: {missing_controllers}")
            return [controller for controller in controllers if controller.name in whitelist]

    def add_joint_velocity_controller(self, namespaces: List[str]):
        """
        For closed loop mode. Tell Giskard how it can send velocities to joints.
        :param namespaces: A list of namespaces where Giskard can find the topics and rosparams.
        """
        self.tree.control_loop_branch.send_controls.add_joint_velocity_controllers(namespaces)

    def add_joint_velocity_group_controller(self, cmd_topic: str, joints: List[str]):
        """
        For closed loop mode. Tell Giskard how it can send velocities for a group of joints.
        """
        internal_joint_names: List[PrefixName] = []
        for i in range(len(joints)):
            internal_joint_names.append(god_map.world.search_for_joint_name(joints[i]))
        self.tree.control_loop_branch.send_controls.add_joint_velocity_group_controllers(cmd_topic=cmd_topic,
                                                                                         joints=internal_joint_names)


class StandAloneRobotInterfaceConfig(RobotInterfaceConfig):
    joint_names: List[str]

    def __init__(self, joint_names: List[str]):
        self.joint_names = joint_names

    def setup(self):
        self.register_controlled_joints(self.joint_names)
