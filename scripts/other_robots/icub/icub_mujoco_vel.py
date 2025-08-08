#!/usr/bin/env python
import rospy

from giskardpy.middleware import set_middleware
from giskardpy.model.collision_avoidance_config import DisableCollisionAvoidanceConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig, SupportedQPSolver

from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.ros1.interface import ROS1Wrapper
from giskardpy_ros.ros1.visualization_mode import VisualizationMode

from giskardpy_ros.configs.other_robots.icub import WorldWithICubConfig, ICubVelocityIAIInterface

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    giskard = Giskard(world_config=WorldWithICubConfig(),
                      collision_avoidance_config=DisableCollisionAvoidanceConfig(),
                      robot_interface_config=ICubVelocityIAIInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(debug_mode=False,
                                                              visualization_mode=VisualizationMode.VisualsFrameLocked),
                      qp_controller_config=QPControllerConfig(mpc_dt=0.05))
    giskard.live()
