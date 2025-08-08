import numpy as np
import rospy

from giskardpy.data_types.data_types import Derivatives, PrefixName
from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.joints import JustinTorso, RevoluteJoint
from giskardpy.model.world_config import WorldConfig
from giskardpy_ros.configs.robot_interface_config import StandAloneRobotInterfaceConfig, RobotInterfaceConfig


class WorldWithICubConfig(WorldConfig):
    map_name: PrefixName
    localization_joint_name: PrefixName
    odom_link_name: PrefixName
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 description_name: str = 'robot_description'):
        super().__init__()
        self.map_name = PrefixName(map_name)
        self.urdf = rospy.get_param(description_name)

    def setup(self):
        self.set_default_color(1, 1, 1, 1)
        self.set_default_limits({Derivatives.velocity: 1,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: None})
        self.add_empty_link(self.map_name)
        self.add_robot_urdf(self.urdf)
        root_link_name = self.get_root_link_of_group(self.robot_group_name)
        self.add_fixed_joint(parent_link=self.map_name, child_link=root_link_name,
                             homogenous_transform=np.array([[1., 0., 0., -0.8],
                                                            [0., 1., 0., 0.],
                                                            [0., 0., 1., 0.550],
                                                            [0., 0., 0., 1.]]))


class ICubVelocityIAIInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map'):
        self.map_name = map_name

    def setup(self):
        self.sync_joint_state_topic('/world/iCub/joint_states')
        self.add_joint_velocity_group_controller(namespace='world/iCub/left_arm_velocity_controller')
        self.add_joint_velocity_group_controller(namespace='world/iCub/right_arm_velocity_controller')
        self.add_joint_velocity_group_controller(namespace='world/iCub/torso_velocity_controller')
