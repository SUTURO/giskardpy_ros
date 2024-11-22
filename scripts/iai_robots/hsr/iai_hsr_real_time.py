#!/usr/bin/env python
import rospy
from giskardpy.middleware import set_middleware

from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.hsr import WorldWithHSRConfig, HSRCollisionAvoidanceConfig, \
    HSRVelocityInterface
from giskardpy_ros.ros1.interface import ROS1Wrapper

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    debug_mode = rospy.get_param('~debug_mode', False)
    giskard = Giskard(world_config=WorldWithHSRConfig(),
                      collision_avoidance_config=HSRCollisionAvoidanceConfig(),
                      robot_interface_config=HSRVelocityInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(publish_free_variables=True, debug_mode=debug_mode,
                                                              add_tf_pub=True))
    giskard.live()
