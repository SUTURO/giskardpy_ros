#!/usr/bin/env python
import rospy

from giskardpy.middleware import set_middleware
from giskardpy.qp.qp_controller_config import QPControllerConfig, SupportedQPSolver
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.hsr import WorldWithHSRConfig, HSRCollisionAvoidanceConfig, \
    HSRMujocoVelocityInterface
from giskardpy_ros.ros1.interface import ROS1Wrapper

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    debug_mode = rospy.get_param('~debug_mode', False)
    giskard = Giskard(world_config=WorldWithHSRConfig(),
                      collision_avoidance_config=HSRCollisionAvoidanceConfig(),
                      robot_interface_config=HSRMujocoVelocityInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(debug_mode=debug_mode),
                      qp_controller_config=QPControllerConfig(max_trajectory_length=300,
                                                              qp_solver=SupportedQPSolver.gurobi))
    giskard.live()
