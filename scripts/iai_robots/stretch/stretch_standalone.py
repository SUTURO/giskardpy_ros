#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.stretch import StretchCollisionAvoidanceConfig, StretchStandaloneInterface
from giskardpy.model.world_config import WorldWithDiffDriveRobot


if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(),
                      collision_avoidance_config=StretchCollisionAvoidanceConfig(),
                      robot_interface_config=StretchStandaloneInterface(),
                      behavior_tree_config=StandAloneBTConfig(publish_tf=True, publish_js=True, debug_mode=True))
    giskard.live()
