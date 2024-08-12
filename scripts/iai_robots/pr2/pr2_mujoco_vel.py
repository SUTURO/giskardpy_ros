from giskardpy_ros.ros2 import rospy
from giskardpy.model.collision_avoidance_config import DisableCollisionAvoidanceConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.pr2 import WorldWithPR2Config, PR2VelocityMujocoInterface


def main():
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithPR2Config(),
                      collision_avoidance_config=DisableCollisionAvoidanceConfig(),
                      robot_interface_config=PR2VelocityMujocoInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(),
                      qp_controller_config=QPControllerConfig())
    giskard.live()


if __name__ == '__main__':
    main()
