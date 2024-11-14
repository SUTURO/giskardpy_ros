#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.stretch import StretchCollisionAvoidanceConfig, StretchStandaloneInterface
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from giskardpy.qp.qp_controller_config import QPControllerConfig, SupportedQPSolver
from giskardpy_ros.ros1.ros_msg_visualization import VisualizationMode

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(),
                      collision_avoidance_config=StretchCollisionAvoidanceConfig(),
                      robot_interface_config=StretchStandaloneInterface(),
                      behavior_tree_config=StandAloneBTConfig(publish_js=False, publish_tf=True, simulation_max_hz=20,
                                                              debug_mode=True,
                                                              visualization_mode=VisualizationMode.CollisionsDecomposedFrameLocked),
                      qp_controller_config=QPControllerConfig(qp_solver=SupportedQPSolver.qpSWIFT))
    giskard.live()
