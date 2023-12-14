#!/usr/bin/env python
import rospy

from giskardpy.configs.giskard import Giskard
from giskardpy.configs.iai_robots.hsr import WorldWithHSRConfig, HSRCollisionAvoidanceConfig, \
    HSRJointTrajInterfaceConfig
from giskardpy.configs.behavior_tree_config import JSConfig


if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithHSRConfig(),
                      collision_avoidance_config=HSRCollisionAvoidanceConfig(),
                      robot_interface_config=HSRJointTrajInterfaceConfig(),
                      behavior_tree_config=JSConfig(publish_js=True))
    giskard.live()
