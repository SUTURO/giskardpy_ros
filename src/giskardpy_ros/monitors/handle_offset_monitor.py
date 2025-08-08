from copy import deepcopy
from typing import Optional

import giskardpy.casadi_wrapper as cas
from geometry_msgs.msg import Vector3Stamped
from giskardpy.data_types.data_types import PrefixName, ObservationState
from giskardpy.god_map import god_map
from giskardpy.model.joints import Joint6DOF
from giskardpy.motion_statechart.monitors.monitors import PayloadMonitor
from rospy import Subscriber

from giskardpy_ros.ros1 import msg_converter


class HandleOffsetCorrection(PayloadMonitor):

    def __init__(self,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 goal_vector: cas.Vector3,
                 door_move_joint: PrefixName,
                 threshold: float = 50,
                 name: Optional[str] = None,
                 magic: float = 20):
        if name is None:
            name = f'{self.__class__.__name__}'
        super().__init__(name=name,
                         run_call_in_thread=False)
        self.root = root_link
        self.tip = tip_link
        self.threshold = threshold
        self.magic = magic
        self.door_move_joint = door_move_joint

        self.root_V_goal_point = god_map.world.transform(self.root, goal_vector)
        self.sub_offset = Subscriber(name='/robokudo/handle_offset', data_class=Vector3Stamped, callback=self.cb)

    def __call__(self, *args, **kwargs):
        root_V_goal_point = deepcopy(self.root_V_goal_point)

        root_V_goal_point.scale(1)
        root_V_goal_point.reference_frame = self.root
        root_V_goal_point.vis_frame = self.tip

        root_P_tip = god_map.world.compose_fk_expression(self.root, self.tip).to_position()

        root_P_goal_point = root_P_tip + root_V_goal_point

        root_V_error = (root_P_goal_point - root_P_tip) / self.magic

        j: Joint6DOF = god_map.world.get_joint(self.door_move_joint)
        assert isinstance(j, Joint6DOF)
        parent_V_error = god_map.world.transform(j.parent_link_name, root_V_error)
        parent_T_child = god_map.world.compute_fk_np(j.parent_link_name, j.child_link_name)
        parent_T_child[:3, 3] += parent_V_error.to_np()[:3]
        parent_T_child = cas.TransMatrix(parent_T_child, reference_frame=j.parent_link_name)
        god_map.world.joints[self.door_move_joint].update_transform(parent_T_child)

        norm = self.root_V_goal_point.norm().to_np()

        self.state = ObservationState.true \
            if norm <= self.threshold \
            else ObservationState.false

    def cb(self, data):
        data = msg_converter.ros_msg_to_giskard_obj(data, god_map.world)
        data = god_map.world.transform(self.root, data)
        self.root_V_goal_point = data


class OffsetCorrectionReset(PayloadMonitor):

    def __init__(self,
                 reset_joint: PrefixName,
                 parent_T_child: cas.TransMatrix,
                 child_frame: PrefixName,
                 name: Optional[str] = None):
        if name is None:
            name = f'{self.__class__.__name__}'
        super().__init__(name=name,
                         run_call_in_thread=False)

        parent_T_child.child_frame = child_frame

        self.reset_joint = reset_joint
        self.parent_T_child = parent_T_child

    def __call__(self, *args, **kwargs):
        moveable_door_joint: Joint6DOF = god_map.world.joints[
            god_map.world.search_for_joint_name(self.reset_joint.short_name)]
        moveable_door_joint.update_transform(self.parent_T_child)

        self.state = ObservationState.true
