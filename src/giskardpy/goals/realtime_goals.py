from __future__ import division

from typing import Optional, List

import rospy
from geometry_msgs.msg import Vector3Stamped, PointStamped, PoseStamped

from giskardpy.monitors.monitors import ExpressionMonitor
from giskardpy.tasks.task import WEIGHT_BELOW_CA
from giskardpy.goals.pointing import Pointing
import giskardpy.casadi_wrapper as cas
from giskardpy.utils.expression_definition_utils import transform_msg


class RealTimePointing(Pointing):

    def __init__(self,
                 tip_link: str,
                 root_link: str,
                 topic_name: str,
                 tip_group: Optional[str] = None,
                 root_group: Optional[str] = None,
                 pointing_axis: Vector3Stamped = None,
                 max_velocity: float = 0.3,
                 weight: float = WEIGHT_BELOW_CA,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.TrueSymbol
                 ):
        initial_goal = PointStamped()
        initial_goal.header.frame_id = 'base_footprint'
        initial_goal.point.x = 1
        initial_goal.point.z = 1
        super().__init__(tip_link=tip_link,
                         goal_point=initial_goal,
                         root_link=root_link,
                         pointing_axis=pointing_axis)

        self.sub = rospy.Subscriber(topic_name, PointStamped, self.cb)

    def cb(self, data: PointStamped):
        data = transform_msg(self.root, data)
        self.root_P_goal_point = data


class RealTimePointingPose(Pointing):

    def __init__(self,
                 tip_link: str,
                 root_link: str,
                 topic_name: str,
                 tip_group: Optional[str] = None,
                 root_group: Optional[str] = None,
                 pointing_axis: Vector3Stamped = None,
                 max_velocity: float = 0.3,
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.TrueSymbol,
                 weight: float = WEIGHT_BELOW_CA):
        initial_goal = PointStamped()
        initial_goal.header.frame_id = 'base_footprint'
        initial_goal.point.x = 1
        initial_goal.point.z = 1
        super().__init__(tip_link=tip_link,
                         goal_point=initial_goal,
                         name=name,
                         root_link=root_link,
                         start_condition=start_condition,
                         hold_condition=hold_condition,
                         end_condition=end_condition,
                         pointing_axis=pointing_axis)

        self.sub = rospy.Subscriber(topic_name, PoseStamped, self.cb)

    def cb(self, data: PoseStamped):
        point_data = PointStamped()
        point_data.point = data.pose.position
        point_data.header = data.header
        # FIXME: Should be removed after Perception fixed frame_id to be without "/" in front or fixed otherwise
        if point_data.header.frame_id.startswith('/'):
            point_data.header.frame_id = point_data.header.frame_id[1:]
        point_data = transform_msg(self.root, point_data)
        self.root_P_goal_point = point_data
