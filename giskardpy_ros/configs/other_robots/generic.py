from typing import Optional, Tuple

import numpy as np
from nav_msgs.msg import Odometry

import giskardpy_ros.ros2.tfwrapper as tf
from giskardpy.data_types.data_types import Derivatives, PrefixName
from giskardpy.data_types.exceptions import UnknownLinkException
from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy.model.world_config import WorldConfig
from giskardpy_ros.configs.giskard import RobotInterfaceConfig
from giskardpy_ros.ros2 import ros2_interface, rospy
from giskardpy_ros.ros2.ros2_interface import wait_for_message, search_for_unique_publisher_of_type
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard


class GenericWorldConfig(WorldConfig):
    robot_name: str = ''
    robot_description: Optional[str]
    controller_manager_name: str

    def __init__(self,
                 robot_description: Optional[str] = None,
                 controller_manager_name: str = 'controller_manager'):
        super().__init__()
        self.robot_description = robot_description
        self.controller_manager_name = controller_manager_name

    @profile
    def get_tf_root_that_is_not_in_world(self) -> str:
        tf_roots = set(tf.get_tf_roots())
        if len(tf_roots) == 1:
            return tf_roots.pop()
        frames_not_in_world = tf_roots.difference(self.world.link_names_as_set)
        if len(frames_not_in_world) > 0:
            return frames_not_in_world.pop()
        return self.world.groups[list(self.world.group_names)[0]].root_link_name.short_name

    @profile
    def setup(self):
        self.set_default_limits({Derivatives.velocity: 0.2,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: 30})
        global_tf_frame = self.get_tf_root_that_is_not_in_world()
        self.map_name = PrefixName(global_tf_frame)
        self.urdf = self.robot_description or ros2_interface.get_robot_description()
        self.add_robot_urdf(self.urdf, self.robot_name)

        root_link_name = self.get_root_link_of_group(self.robot_name)
        # gather frames between tf root and robot root
        chain = tf.get_frame_chain(global_tf_frame, root_link_name.short_name)
        # add all missing frames to world
        for i, tf_frame in enumerate(chain):
            try:
                world_link_name = self.world.search_for_link_name(tf_frame)
            except UnknownLinkException as e:
                world_link_name = PrefixName(tf_frame)
                self.add_empty_link(world_link_name)
            chain[i] = world_link_name
        odom_frames = self.has_odom()
        for link1, link2 in zip(chain, chain[1:]):
            if (link1, link2) == odom_frames:
                self.add_omni_drive_joint(name='brumbrum',
                                          parent_link_name=link1,
                                          child_link_name=link2,
                                          translation_limits={
                                              Derivatives.velocity: 0.2,
                                              Derivatives.acceleration: 1,
                                              Derivatives.jerk: 5,
                                          },
                                          rotation_limits={
                                              Derivatives.velocity: 0.2,
                                              Derivatives.acceleration: 1,
                                              Derivatives.jerk: 5
                                          },
                                          robot_group_name=self.robot_group_name)
            else:
                self.add_fixed_joint(parent_link=link1, child_link=link2)

    def has_odom(self) -> Optional[Tuple[str, str]]:
        try:
            topic_name = search_for_unique_publisher_of_type(Odometry)
            message: Odometry = wait_for_message(Odometry, rospy.node, topic_name, qos_profile=10)[1]
            return message.header.frame_id, message.child_frame_id
        except Exception as e:
            return None


class GenericRobotInterface(RobotInterfaceConfig):
    drive_joint_name: str

    def __init__(self, controller_manager_name: str = 'controller_manager'):
        self.controller_manager_name = controller_manager_name

    def setup(self):
        if GiskardBlackboard().tree.is_standalone():
            self.register_controlled_joints(self.world.movable_joint_names)
        elif GiskardBlackboard().tree.is_closed_loop():
            self.discover_interfaces_from_controller_manager()
            try:
                self.world.get_drive_joint()
                self.sync_odometry_topic()
                self.add_base_cmd_velocity()
            except ValueError as e:
                # no drive joint, so no need to add odom and cmd vel topic
                pass
        else:
            raise NotImplementedError('this mode is not implemented yet')


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
