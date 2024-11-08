#!/usr/bin/env python
import rospy

from giskardpy.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy.configs.giskard import Giskard
from giskardpy.configs.iai_robots.donbot import WorldWithBoxyBaseConfig, DonbotCollisionAvoidanceConfig, \
    DonbotVelocityInterfaceConfig

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithBoxyBaseConfig(),
                      collision_avoidance_config=DonbotCollisionAvoidanceConfig(),
                      robot_interface_config=DonbotVelocityInterfaceConfig(),
                      behavior_tree_config=ClosedLoopBTConfig())
    giskard.live()
