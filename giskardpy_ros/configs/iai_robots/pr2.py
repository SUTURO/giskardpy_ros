from typing import Optional

import numpy as np

from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.world_config import WorldWithOmniDriveRobot
from giskardpy_ros.configs.giskard import RobotInterfaceConfig
from giskardpy.data_types.data_types import Derivatives, PrefixName
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy_ros.configs.other_robots.generic import GenericWorldConfig
from giskardpy_ros.ros2 import ros2_interface


class WorldWithPR2Config(GenericWorldConfig):
    def __init__(self, localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined', drive_joint_name: str = 'brumbrum',
                 urdf: Optional[str] = None):
        super().__init__()
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name
        self.robot_description = urdf

    def setup(self):
        self.set_default_limits({Derivatives.velocity: 0.2,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: 30})
        self.map_name = PrefixName(self.get_tf_root_that_is_not_in_world())
        self.add_empty_link(self.map_name)
        self.urdf = self.robot_description or ros2_interface.get_robot_description()
        self.add_robot_urdf(self.urdf, self.robot_name)


        root_link_name = self.get_root_link_of_group(self.robot_name)

        self.add_omni_drive_joint(name=self.drive_joint_name,
                                  parent_link_name=self.map_name,
                                  child_link_name=root_link_name,
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

        self.set_joint_limits(limit_map={Derivatives.velocity: 2,
                                         Derivatives.jerk: 60},
                              joint_name='head_pan_joint')
        self.set_joint_limits(limit_map={Derivatives.velocity: 4,
                                         Derivatives.jerk: 120},
                              joint_name='head_tilt_joint')


class PR2StandaloneInterface(RobotInterfaceConfig):
    drive_joint_name: str

    def __init__(self, drive_joint_name: str):
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.register_controlled_joints([
            'torso_lift_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'r_shoulder_pan_joint',
            'r_shoulder_lift_joint',
            'r_upper_arm_roll_joint',
            'r_forearm_roll_joint',
            'r_elbow_flex_joint',
            'r_wrist_flex_joint',
            'r_wrist_roll_joint',
            'l_shoulder_pan_joint',
            'l_shoulder_lift_joint',
            'l_upper_arm_roll_joint',
            'l_forearm_roll_joint',
            'l_elbow_flex_joint',
            'l_wrist_flex_joint',
            'l_wrist_roll_joint',
            self.drive_joint_name,
        ])


class PR2JointTrajServerMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/joint_states')
        self.sync_odometry_topic('/pr2/base_footprint', self.drive_joint_name)
        self.add_follow_joint_trajectory_server(
            namespace='/pr2/whole_body_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/pr2/l_gripper_l_finger_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/pr2/r_gripper_l_finger_controller')
        self.add_base_cmd_velocity(cmd_vel_topic='/pr2/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)


class PR2VelocityMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.discover_interfaces_from_controller_manager()
        self.sync_odometry_topic('/odom', self.drive_joint_name)
        self.add_base_cmd_velocity(cmd_vel_topic='/cmd_vel')


class PR2CollisionAvoidance(CollisionAvoidanceConfig):
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
