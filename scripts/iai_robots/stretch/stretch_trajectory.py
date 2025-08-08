#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import OpenLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.stretch import StretchCollisionAvoidanceConfig, StretchTrajectoryInterface
from giskardpy_ros.configs.iai_robots.tiago import TiagoCollisionAvoidanceConfig, TiagoStandaloneInterface
from giskardpy.model.world_config import WorldWithDiffDriveRobot

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(urdf=rospy.get_param('robot_description')),
                      collision_avoidance_config=StretchCollisionAvoidanceConfig(),
                      robot_interface_config=StretchTrajectoryInterface(),
                      behavior_tree_config=OpenLoopBTConfig(debug_mode=True),
                      )
    giskard.live()