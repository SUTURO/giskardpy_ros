#!/usr/bin/env python
import rospy

from giskardpy.middleware import set_middleware
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.tiago import TiagoCollisionAvoidanceConfig, TiagoStandaloneInterface
from giskardpy_ros.ros1.interface import ROS1Wrapper

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(urdf=rospy.get_param('robot_description')),
                      collision_avoidance_config=TiagoCollisionAvoidanceConfig(),
                      robot_interface_config=TiagoStandaloneInterface(),
                      behavior_tree_config=StandAloneBTConfig())
    giskard.live()
