import numpy as np
import rospy

from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.world_config import WorldConfig
from giskardpy_ros.configs.robot_interface_config import StandAloneRobotInterfaceConfig, RobotInterfaceConfig
from giskardpy.data_types.data_types import PrefixName, Derivatives
from giskardpy.god_map import god_map
from giskardpy_ros.ros1 import tfwrapper as tf, msg_converter
from giskardpy.middleware import get_middleware


class WorldWithHSRConfig(WorldConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum',
                 description_name: str = 'robot_description'):
        super().__init__()
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name
        self.robot_description_name = description_name

    def setup(self):
        self.set_default_color(1, 1, 1, 1)
        self.set_default_limits({Derivatives.velocity: 1,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: None})
        self.add_empty_link(PrefixName(self.map_name))
        self.add_6dof_joint(parent_link=self.map_name, child_link=self.odom_link_name,
                            joint_name=self.localization_joint_name)
        self.add_empty_link(PrefixName(self.odom_link_name))
        self.add_robot_urdf(urdf=rospy.get_param(self.robot_description_name))

        try:
            self.check_for_link_name(link_name='hand_gripper_tool_frame')
        except ValueError as e:
            get_middleware().logwarn(f'Could not find Hand Gripper Tool Frame Exception: {e}')
        else:
            get_middleware().loginfo(f'Hand Gripper Tool Frame Found')

        root_link_name = self.get_root_link_of_group(self.robot_group_name)
        self.add_omni_drive_joint(parent_link_name=self.odom_link_name,
                                  child_link_name=root_link_name,
                                  name=self.drive_joint_name,
                                  x_name=PrefixName('odom_x', self.robot_group_name),
                                  y_name=PrefixName('odom_y', self.robot_group_name),
                                  yaw_vel_name=PrefixName('odom_t', self.robot_group_name),
                                  translation_limits={
                                      Derivatives.velocity: 0.2,
                                      Derivatives.acceleration: np.inf,
                                      Derivatives.jerk: None,
                                  },
                                  rotation_limits={
                                      Derivatives.velocity: 0.2,
                                      Derivatives.acceleration: np.inf,
                                      Derivatives.jerk: None
                                  },
                                  robot_group_name=self.robot_group_name)
        self.world.register_group(name='gripper',
                                  root_link_name=self.world.search_for_link_name('wrist_roll_link'),
                                  actuated=False)


class SuturoArenaWithHSRConfig(WorldWithHSRConfig):

    def __init__(self, map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum',
                 description_name: str = 'robot_description',
                 environment_description: str = 'kitchen_description',
                 environment_name: str = 'iai_kitchen'):
        super().__init__(map_name, localization_joint_name, odom_link_name, drive_joint_name, description_name)
        self.environment_name = environment_description
        self.kitchen_name = environment_name

    def setup(self):
        super().setup()
        urdf = rospy.get_param(self.environment_name)
        god_map.world.add_urdf(urdf=urdf,
                               group_name=self.kitchen_name,
                               actuated=False)
        root_link_name = self.get_root_link_of_group(self.kitchen_name)
        kitchen_pose = tf.lookup_pose(self.map_name, 'iai_kitchen/urdf_main')
        self.add_fixed_joint(parent_link=self.map_name, child_link=root_link_name,
                             homogenous_transform=msg_converter.ros_msg_to_giskard_obj(kitchen_pose.pose, god_map.world))


class HSRCollisionAvoidanceConfig(CollisionAvoidanceConfig):
    def __init__(self, drive_joint_name: str = 'brumbrum'):
        super().__init__()
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.load_self_collision_matrix('package://giskardpy_ros/self_collision_matrices/iai/hsrb.srdf')
        self.set_default_external_collision_avoidance(soft_threshold=0.05,
                                                      hard_threshold=0.0)
        self.overwrite_external_collision_avoidance('wrist_roll_joint',
                                                    number_of_repeller=4,
                                                    soft_threshold=0.05,
                                                    hard_threshold=0.0,
                                                    max_velocity=0.2)
        self.overwrite_external_collision_avoidance(joint_name=self.drive_joint_name,
                                                    number_of_repeller=2,
                                                    soft_threshold=0.1,
                                                    hard_threshold=0.03)
        self.overwrite_self_collision_avoidance(link_name='head_tilt_link',
                                                soft_threshold=0.03)


class HSRStandaloneInterface(StandAloneRobotInterfaceConfig):
    def __init__(self, drive_joint_name: str = 'brumbrum'):
        super().__init__([
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
            drive_joint_name])


class HSRVelocityInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/hsrb/joint_states')
        self.sync_odometry_topic('/hsrb/odom', self.drive_joint_name,
                                 sync_in_control_loop=False)

        self.add_joint_velocity_group_controller(namespace='hsrb/realtime_body_controller_real')

        self.add_base_cmd_velocity(cmd_vel_topic='/hsrb/command_velocity',
                                   joint_name=self.drive_joint_name)


class HSRVelocityInterfaceSuturo(HSRVelocityInterface):

    def __init__(self, map_name: str = 'map', localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom', drive_joint_name: str = 'brumbrum', environment_name: str = 'iai_kitchen'):
        super().__init__(map_name, localization_joint_name, odom_link_name, drive_joint_name)
        self.environment_name = environment_name

    def setup(self):
        super().setup()
        self.sync_joint_state_topic('/iai_kitchen/joint_states', group_name=self.environment_name)


class HSRJointTrajInterfaceConfig(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/hsrb/joint_states')
        self.sync_odometry_topic('/hsrb/odom', self.drive_joint_name)

        self.add_follow_joint_trajectory_server(namespace='/hsrb/head_trajectory_controller',
                                                fill_velocity_values=True)
        self.add_follow_joint_trajectory_server(namespace='/hsrb/arm_trajectory_controller',
                                                fill_velocity_values=True)
        self.add_base_cmd_velocity(cmd_vel_topic='/hsrb/command_velocity',
                                   track_only_velocity=False,
                                   joint_name=self.drive_joint_name)
        # self.add_follow_joint_trajectory_server(namespace='/hsrb/omni_base_controller',
        #                                         fill_velocity_values=True,
        #                                         path_tolerance={
        #                                             Derivatives.position: 1,
        #                                             Derivatives.velocity: 1,
        #                                             Derivatives.acceleration: 100})
        # self.add_base_cmd_velocity(cmd_vel_topic='/hsrb/command_velocity',
        #                            track_only_velocity=True,
        #                            joint_name=self.drive_joint_name)


class HSRMujocoVelocityInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/hsrb4s/joint_states')
        self.sync_odometry_topic('/hsrb4s/base_footprint', self.drive_joint_name)

        self.add_joint_velocity_controller(namespaces=['hsrb4s/arm_flex_joint_velocity_controller',
                                                       'hsrb4s/arm_lift_joint_velocity_controller',
                                                       'hsrb4s/arm_roll_joint_velocity_controller',
                                                       'hsrb4s/head_pan_joint_velocity_controller',
                                                       'hsrb4s/head_tilt_joint_velocity_controller',
                                                       'hsrb4s/wrist_flex_joint_velocity_controller',
                                                       'hsrb4s/wrist_roll_joint_velocity_controller'])

        self.add_base_cmd_velocity(cmd_vel_topic='/hsrb4s/cmd_vel',
                                   joint_name=self.drive_joint_name)


class HSRMujocoPositionInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/hsrb4s/joint_states')
        self.sync_odometry_topic('/hsrb4s/base_footprint', self.drive_joint_name)

        self.add_joint_position_controller(namespaces=[
            'hsrb4s/arm_flex_joint_position_controller',
            # 'hsrb4s/arm_lift_joint_position_controller',
            'hsrb4s/arm_roll_joint_position_controller',
            'hsrb4s/head_pan_joint_position_controller',
            'hsrb4s/head_tilt_joint_position_controller',
            'hsrb4s/wrist_flex_joint_position_controller',
            'hsrb4s/wrist_roll_joint_position_controller'
        ])

        self.add_joint_velocity_controller(namespaces=[
            # 'hsrb4s/arm_flex_joint_position_controller',
            'hsrb4s/arm_lift_joint_position_controller',
            # 'hsrb4s/arm_roll_joint_position_controller',
            # 'hsrb4s/head_pan_joint_position_controller',
            # 'hsrb4s/head_tilt_joint_position_controller',
            # 'hsrb4s/wrist_flex_joint_position_controller',
            # 'hsrb4s/wrist_roll_joint_position_controller'
        ])

        self.add_base_cmd_velocity(cmd_vel_topic='/hsrb4s/cmd_vel',
                                   joint_name=self.drive_joint_name)
