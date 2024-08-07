from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException

from giskardpy_ros.ros2 import rospy
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig

from giskardpy.model.collision_avoidance_config import DefaultCollisionAvoidanceConfig, DisableCollisionAvoidanceConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.generic import GenericWorldConfig, GenericRobotInterface
from giskardpy_ros.ros2.ros2_interface import load_urdf


def main():
    rospy.init_node('giskard')
    try:
        rospy.node.declare_parameters(namespace='',
                                      parameters=[('robot_description', Parameter.Type.STRING)])
        robot_description = rospy.node.get_parameter('robot_description').value
    except ParameterUninitializedException as e:
        robot_description = load_urdf('package://iai_pr2_description/robots/pr2_with_ft2_cableguide.xacro')
    giskard = Giskard(world_config=GenericWorldConfig(robot_description=robot_description),
                      collision_avoidance_config=DisableCollisionAvoidanceConfig(),
                      robot_interface_config=GenericRobotInterface(),
                      behavior_tree_config=StandAloneBTConfig(publish_js=True,
                                                              publish_tf=False,
                                                              debug_mode=True),
                      qp_controller_config=QPControllerConfig())
    giskard.live()


if __name__ == '__main__':
    main()
