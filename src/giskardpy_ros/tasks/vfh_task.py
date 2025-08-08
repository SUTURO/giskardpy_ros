from typing import Optional

import giskardpy.casadi_wrapper as cas
from giskardpy.data_types.data_types import PrefixName, ColorRGBA
from giskardpy.motion_statechart.tasks.task import Task, WEIGHT_COLLISION_AVOIDANCE
from giskardpy.god_map import god_map


class VFHMoveDir(Task):
    def __init__(self,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 laser_link: PrefixName,
                 laser_V_goal_angle: cas.Vector3,
                 max_velocity: float = 0.3,
                 weight: float = WEIGHT_COLLISION_AVOIDANCE,
                 name: Optional[str] = None):
        """
        The VFH MoveDir task pushes the robot in a given direction calculated by
        the Vector Field Histogram Algorithm.

        :param root_link: prefix name of the root link
        :param tip_link: prefix name of the tip link
        :param laser_link: prefix name of the laser link
        :param laser_V_goal_angle: vector of the angle between the laser scanner and the goal
        :param max_velocity: maximum velocity of the robot
        :param weight: weight of the movement
        :param name: name of the task
        """
        self.tip_link = tip_link
        self.root = root_link
        self.max_velocity = max_velocity
        self.weight = weight

        if name is None:
            name = f'{self.__class__.__name__}/{self.root}/{self.tip_link}'
        super().__init__(name=name)
        laser_V_goal_angle.vis_frame = tip_link

        root_T_tip = god_map.world.compose_fk_expression(self.root, self.tip_link)
        root_T_laser = god_map.world.compose_fk_expression(self.root, laser_link)
        root_V_goal_angle = root_T_laser.dot(laser_V_goal_angle)
        root_P_tip = root_T_tip.to_position()
        root_P_goal_point = root_P_tip + root_V_goal_angle

        god_map.debug_expression_manager.add_debug_expression(name="root_P_goal_point",
                                                              expression=root_P_goal_point,
                                                              color=ColorRGBA(1.0, 1.0, 1.0, 1.0))
        god_map.debug_expression_manager.add_debug_expression(name="root_V_goal_angle",
                                                              expression=root_V_goal_angle,
                                                              color=ColorRGBA(0.0, 1.0, 1.0, 1.0))

        self.add_point_goal_constraints(frame_P_current=root_P_tip, frame_P_goal=root_P_goal_point,
                                        reference_velocity=self.max_velocity, weight=self.weight)


# class RealMoveDir(VFHMoveDir):
#     def __init__(self,
#                  tip_link: PrefixName,
#                  root_link: PrefixName,
#                  topic_name: str,
#                  max_velocity: float = 0.3,
#                  weight: float = WEIGHT_BELOW_CA,
#                  name: Optional[str] = None):
#         initial_goal = cas.Vector3((0, 0, 0), reference_frame=god_map.world.search_for_link_name(tip_link))
#         super().__init__(name=name,
#                          tip_link=tip_link,
#                          goal_vector=initial_goal,
#                          root_link=root_link,
#                          max_velocity=max_velocity,
#                          weight=weight)
#         self.sub = rospy.Subscriber(topic_name, Vector3Stamped, self.cb, queue_size=10)
#
#     def cb(self, data: Vector3Stamped):
#         data = msg_converter.ros_msg_to_giskard_obj(data, god_map.world)
#         data = god_map.world.transform(self.root, data).to_np()
#         self.root_V_goal_angle = data
