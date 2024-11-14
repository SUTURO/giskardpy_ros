#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.tiago import TiagoCollisionAvoidanceConfig, TiagoStandaloneInterface
from giskardpy.model.world_config import WorldWithDiffDriveRobot

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(),
                      collision_avoidance_config=TiagoCollisionAvoidanceConfig(),
                      robot_interface_config=TiagoStandaloneInterface(),
                      behavior_tree_config=StandAloneBTConfig())
    giskard.live()
