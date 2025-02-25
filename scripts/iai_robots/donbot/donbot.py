#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.donbot import WorldWithBoxyBaseConfig, DonbotCollisionAvoidanceConfig, \
    DonbotJointTrajInterfaceConfig

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithBoxyBaseConfig(),
                      collision_avoidance_config=DonbotCollisionAvoidanceConfig(),
                      robot_interface_config=DonbotJointTrajInterfaceConfig())
    giskard.live()
