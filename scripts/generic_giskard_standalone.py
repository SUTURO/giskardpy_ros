from giskardpy_ros.ros2 import rospy
from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException

from giskardpy.model.collision_avoidance_config import DisableCollisionAvoidanceConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.generic import GenericWorldConfig, GenericRobotInterface


def main():
    rospy.init_node('giskard')
    try:
        rospy.node.declare_parameters(namespace='',
                                      parameters=[('robot_description', Parameter.Type.STRING)])
        robot_description = rospy.node.get_parameter_or('robot_description').value
    except ParameterUninitializedException as e:
        robot_description = None
    giskard = Giskard(world_config=GenericWorldConfig(robot_description=robot_description),
                      collision_avoidance_config=DisableCollisionAvoidanceConfig(),
                      robot_interface_config=GenericRobotInterface(),
                      behavior_tree_config=StandAloneBTConfig(publish_free_variables=True,
                                                              publish_tf=False,
                                                              debug_mode=True),
                      qp_controller_config=QPControllerConfig())
    giskard.live()


if __name__ == '__main__':
    main()
