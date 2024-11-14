#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.pr2 import PR2CollisionAvoidance, WorldWithPR2Config, PR2VelocityMujocoInterface
from giskardpy.qp.qp_controller_config import QPControllerConfig, SupportedQPSolver

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithPR2Config(),
                      collision_avoidance_config=PR2CollisionAvoidance(),
                      robot_interface_config=PR2VelocityMujocoInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(debug_mode=True),
                      qp_controller_config=QPControllerConfig(qp_solver=SupportedQPSolver.gurobi))
    giskard.live()
