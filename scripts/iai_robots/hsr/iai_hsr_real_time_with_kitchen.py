#!/usr/bin/env python
import rospy
from giskardpy. middleware import set_middleware
from giskardpy.qp.qp_controller_config import QPControllerConfig

from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.hsr import HSRCollisionAvoidanceConfig, \
    SuturoArenaWithHSRConfig, HSRVelocityInterfaceSuturo
from giskardpy_ros.ros1.interface import ROS1Wrapper

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    debug_mode = rospy.get_param('~debug_mode', False)
    debug_mode = True
    environment_name = 'iai_kitchen'
    giskard = Giskard(world_config=SuturoArenaWithHSRConfig(environment_name=environment_name),
                      collision_avoidance_config=HSRCollisionAvoidanceConfig(),
                      robot_interface_config=HSRVelocityInterfaceSuturo(environment_name=environment_name),
                      behavior_tree_config=ClosedLoopBTConfig(publish_free_variables=True, debug_mode=debug_mode,
                                                              add_tf_pub=True),
                      qp_controller_config=QPControllerConfig(mpc_dt=0.017,
                                                              prediction_horizon=15,
                                                              control_dt=0.017))  # TODO: test 70 HZ dt + 10 PH
    giskard.live()
