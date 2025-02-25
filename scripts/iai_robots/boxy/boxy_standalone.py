#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.boxy import BoxyCollisionAvoidanceConfig, BoxyStandaloneInterfaceConfig
from giskardpy_ros.configs.iai_robots.donbot import WorldWithBoxyBaseConfig

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithBoxyBaseConfig(),
                      collision_avoidance_config=BoxyCollisionAvoidanceConfig(),
                      robot_interface_config=BoxyStandaloneInterfaceConfig(),
                      behavior_tree_config=StandAloneBTConfig())
    giskard.live()
