#!/usr/bin/env python
import rospy

from giskardpy.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy.configs.giskard import Giskard
from giskardpy.configs.iai_robots.hsr import HSRCollisionAvoidanceConfig, \
    SuturoArenaWithHSRConfig, HSRVelocityInterfaceSuturo

if __name__ == '__main__':
    rospy.init_node('giskard')
    debug_mode = rospy.get_param('~debug_mode', False)
    environment_name = 'iai_kitchen'
    giskard = Giskard(world_config=SuturoArenaWithHSRConfig(environment_name=environment_name),
                      collision_avoidance_config=HSRCollisionAvoidanceConfig(),
                      robot_interface_config=HSRVelocityInterfaceSuturo(environment_name=environment_name),
                      behavior_tree_config=ClosedLoopBTConfig(publish_free_variables=True, debug_mode=debug_mode,
                                                              add_tf_pub=True))
    giskard.live()
