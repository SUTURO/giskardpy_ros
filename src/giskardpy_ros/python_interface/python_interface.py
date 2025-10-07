from __future__ import annotations

import re
from collections import defaultdict
from typing import Dict, Tuple, Optional, List, Union

import giskard_msgs.msg as giskard_msgs
import numpy as np
import rospy
from actionlib import SimpleActionClient
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerResponse, \
    ListControllersResponse
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, QuaternionStamped, Vector3, Quaternion, Point
from giskard_msgs.msg import ExecutionState
from giskard_msgs.msg import (MoveAction, MoveGoal, WorldBody, CollisionEntry, MoveResult, MoveFeedback,
                              WorldGoal, WorldAction, WorldResult, MotionStatechartNode)
from giskard_msgs.srv import DyeGroupRequest, DyeGroup, GetGroupInfoRequest, DyeGroupResponse
from giskard_msgs.srv import GetGroupInfo, GetGroupNames
from giskard_msgs.srv import GetGroupNamesResponse, GetGroupInfoResponse
from giskardpy.data_types.data_types import goal_parameter
from giskardpy.data_types.exceptions import MaxTrajectoryLengthException, \
    MonitorInitalizationException, ObjectForceTorqueThresholdException
from giskardpy.data_types.suturo_types import ForceTorqueThresholds, TakePoseTypes, MoveAroundHingeAlign
from giskardpy.motion_statechart.goals.align_to_push_door import AlignToPushDoor
from giskardpy.motion_statechart.goals.cartesian_goals import DiffDriveBaseGoal, \
    CartesianPoseStraight, CartesianPositionStraight, CartesianPose
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.motion_statechart.goals.move_around_hinge import MoveAroundHinge
from giskardpy.motion_statechart.goals.open_close import Close, Open
from giskardpy.motion_statechart.goals.open_door import OpenDoorGoal
from giskardpy.motion_statechart.goals.pre_push_door import PrePushDoor
from giskardpy.motion_statechart.goals.suturo import Reaching, Placing, Mixing, \
    JointRotationGoalContinuous, Tilting, TakePose, AlignHeight, Retracting, VerticalMotion
from giskardpy.motion_statechart.monitors.cartesian_monitors import PoseReached, PositionReached, OrientationReached, \
    PointingAt, VectorsAligned, DistanceToLine
from giskardpy.motion_statechart.monitors.feature_monitors import PerpendicularMonitor, AngleMonitor, HeightMonitor, \
    DistanceMonitor
from giskardpy.motion_statechart.monitors.force_torque_monitor import PayloadForceTorque
from giskardpy.motion_statechart.monitors.joint_monitors import JointGoalReached, JointPositionAbove
from giskardpy.motion_statechart.monitors.lidar_monitor import LidarPayloadMonitor
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached, TimeAbove, Alternator, CancelMotion, \
    EndMotion, TrueMonitor, FalseMonitor
from giskardpy.motion_statechart.monitors.overwrite_state_monitors import SetOdometry, SetSeedConfiguration
from giskardpy.motion_statechart.monitors.payload_monitors import Print, Sleep, \
    Pulse, CheckMaxTrajectoryLength
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition, CartesianOrientation, \
    JustinTorsoLimitCart, CartesianVelocityLimit
from giskardpy.motion_statechart.tasks.feature_functions import AlignPerpendicular, HeightGoal, AngleGoal, DistanceGoal
from giskardpy.motion_statechart.tasks.grasp_bar import GraspBar, GraspBarOffset
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionLimitList, JointPositionList, AvoidJointLimits
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionListStop
from giskardpy.motion_statechart.tasks.pointing import Pointing
from giskardpy.motion_statechart.tasks.task import WEIGHT_ABOVE_CA
from giskardpy.motion_statechart.tasks.task import WEIGHT_BELOW_CA
from giskardpy.motion_statechart.tasks.weight_scaling_goals import MaxManipulability, BaseArmWeightScaling
from giskardpy.motion_statechart.tasks.wiggle_insert import WiggleInsert
from giskardpy.utils.utils import get_all_classes_in_package, ImmutableDict
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from tf.transformations import quaternion_from_matrix

from giskardpy_ros.goals.carry_my_luggage import CarryMyBullshit
from giskardpy_ros.goals.grasp_with_ft import GraspWithForceTorqueGoal
from giskardpy_ros.monitors.handle_offset_monitor import HandleOffsetCorrection, OffsetCorrectionReset
from giskardpy_ros.ros1 import msg_converter
from giskardpy_ros.ros1 import tfwrapper as tf
from giskardpy_ros.ros1.msg_converter import kwargs_to_json
from giskardpy_ros.tasks.realtime_tasks import RealTimePointing, RealTimeConePointing
from giskardpy_ros.tree.control_modes import ControlModes
from giskardpy_ros.utils.utils import make_world_body_box


class WorldWrapper:
    def __init__(self, node_name: str):
        self._get_group_info_srv = rospy.ServiceProxy(f'{node_name}/get_group_info', GetGroupInfo)
        self._get_group_names_srv = rospy.ServiceProxy(f'{node_name}/get_group_names', GetGroupNames)
        self._dye_group_srv = rospy.ServiceProxy(f'{node_name}/dye_group', DyeGroup)
        self._control_mode_srv = rospy.ServiceProxy(f'{node_name}/get_control_mode', Trigger)
        self._client = SimpleActionClient(f'{node_name}/update_world', WorldAction)
        self._client.wait_for_server()
        rospy.wait_for_service(self._get_group_names_srv.resolved_name)
        self.robot_name = self.get_group_names()[0]

    def clear(self) -> WorldResult:
        """
        Resets the world to what it was when Giskard was launched.
        """
        req = WorldGoal()
        req.operation = WorldGoal.REMOVE_ALL
        return self._send_goal_and_wait(req)

    def remove_group(self, name: str) -> WorldResult:
        """
        Removes a group and all links and joints it contains from the world.
        Be careful, you can remove parts of the robot like that.
        """
        world_body = WorldBody()
        req = WorldGoal()
        req.group_name = str(name)
        req.operation = WorldGoal.REMOVE
        req.body = world_body
        return self._send_goal_and_wait(req)

    def _send_goal_and_wait(self, goal: WorldGoal) -> WorldResult:
        self._client.send_goal_and_wait(goal)
        result: WorldResult = self._client.get_result()
        error = msg_converter.error_msg_to_exception(result.error)
        if error is not None:
            raise error
        else:
            return result

    def add_box(self,
                name: str,
                size: Tuple[float, float, float],
                pose: PoseStamped,
                parent_link: Optional[Union[str, giskard_msgs.LinkName]] = None) -> WorldResult:
        """
        Adds a new box to the world tree and attaches it to parent_link.
        If parent_link_group and parent_link are empty, the box will be attached to the world root link, e.g., map.
        :param name: How the new group will be called
        :param size: X, Y and Z dimensions of the box, respectively
        :param pose: Where the root link of the new object will be positioned
        :param parent_link: Name of the link, the object will get attached to. None = root link of world
        :return: Response message of the service call
        """
        if isinstance(parent_link, str):
            parent_link = giskard_msgs.LinkName(name=parent_link)
        parent_link = parent_link or giskard_msgs.LinkName()
        req = WorldGoal()
        req.group_name = str(name)
        req.operation = WorldGoal.ADD
        req.body = make_world_body_box(size[0], size[1], size[2])
        req.parent_link = parent_link or giskard_msgs.LinkName()
        req.pose = pose
        return self._send_goal_and_wait(req)

    def add_sphere(self,
                   name: str,
                   radius: float,
                   pose: PoseStamped,
                   parent_link: Optional[Union[str, giskard_msgs.LinkName]] = None) -> WorldResult:
        """
        See add_box.
        """
        if isinstance(parent_link, str):
            parent_link = giskard_msgs.LinkName(name=parent_link)
        parent_link = parent_link or giskard_msgs.LinkName()
        world_body = WorldBody()
        world_body.type = WorldBody.PRIMITIVE_BODY
        world_body.shape.type = SolidPrimitive.SPHERE
        world_body.shape.dimensions.append(radius)
        req = WorldGoal()
        req.group_name = str(name)
        req.operation = WorldGoal.ADD
        req.body = world_body
        req.pose = pose
        req.parent_link = parent_link
        return self._send_goal_and_wait(req)

    def add_mesh(self,
                 name: str,
                 mesh: str,
                 pose: PoseStamped,
                 parent_link: Optional[Union[str, giskard_msgs.LinkName]] = None,
                 scale: Tuple[float, float, float] = (1, 1, 1)) -> WorldResult:
        """
        See add_box.
        :param mesh: path to the mesh location, can be ros package path, e.g.,
                        package://giskardpy/test/urdfs/meshes/bowl_21.obj
        """
        if isinstance(parent_link, str):
            parent_link = giskard_msgs.LinkName(name=parent_link)
        parent_link = parent_link or giskard_msgs.LinkName()
        world_body = WorldBody()
        world_body.type = WorldBody.MESH_BODY
        world_body.mesh = mesh
        req = WorldGoal()
        req.group_name = str(name)
        req.operation = WorldGoal.ADD
        req.body = world_body
        req.pose = pose
        req.body.scale.x = scale[0]
        req.body.scale.y = scale[1]
        req.body.scale.z = scale[2]
        req.parent_link = parent_link
        return self._send_goal_and_wait(req)

    def add_cylinder(self,
                     name: str,
                     height: float,
                     radius: float,
                     pose: PoseStamped,
                     parent_link: Optional[Union[str, giskard_msgs.LinkName]] = None) -> WorldResult:
        """
        See add_box.
        """
        if isinstance(parent_link, str):
            parent_link = giskard_msgs.LinkName(name=parent_link)
        parent_link = parent_link or giskard_msgs.LinkName()
        world_body = WorldBody()
        world_body.type = WorldBody.PRIMITIVE_BODY
        world_body.shape.type = SolidPrimitive.CYLINDER
        world_body.shape.dimensions = [0, 0]
        world_body.shape.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = height
        world_body.shape.dimensions[SolidPrimitive.CYLINDER_RADIUS] = radius
        req = WorldGoal()
        req.group_name = str(name)
        req.operation = WorldGoal.ADD
        req.body = world_body
        req.pose = pose
        req.parent_link = parent_link
        return self._send_goal_and_wait(req)

    def update_parent_link_of_group(self,
                                    name: str,
                                    parent_link: Union[str, giskard_msgs.LinkName]) -> WorldResult:
        """
        Removes the joint connecting the root link of a group and attaches it to a parent_link.
        The object will not move relative to the world's root link in this process.
        :param name: name of the group
        :param parent_link: name of the new parent link
        :return: result message
        """
        if isinstance(parent_link, str):
            parent_link = giskard_msgs.LinkName(name=parent_link)
        req = WorldGoal()
        req.operation = WorldGoal.UPDATE_PARENT_LINK
        req.group_name = str(name)
        req.parent_link = parent_link
        return self._send_goal_and_wait(req)

    def detach_group(self, object_name: str) -> WorldResult:
        """
        A wrapper for update_parent_link_of_group which set parent_link to the root link of the world.
        """
        req = WorldGoal()
        req.group_name = str(object_name)
        req.operation = req.UPDATE_PARENT_LINK
        return self._send_goal_and_wait(req)

    def add_urdf(self,
                 name: str,
                 urdf: str,
                 pose: PoseStamped,
                 parent_link: Optional[Union[str, giskard_msgs.LinkName]] = None,
                 js_topic: Optional[str] = '') -> WorldResult:
        """
        Adds an urdf to the world.
        :param name: name the group containing the urdf will have.
        :param urdf: urdf as string, no path!
        :param pose: pose of the root link of the new object
        :param parent_link: to which link the urdf will be attached
        :param js_topic: Giskard will listen on that topic for joint states and update the urdf accordingly
        :return: response message
        """
        if isinstance(parent_link, str):
            parent_link = giskard_msgs.LinkName(name=parent_link)
        parent_link = parent_link or giskard_msgs.LinkName()
        js_topic = str(js_topic)
        urdf_body = WorldBody()
        urdf_body.type = WorldBody.URDF_BODY
        urdf_body.urdf = str(urdf)
        urdf_body.joint_state_topic = js_topic
        req = WorldGoal()
        req.group_name = str(name)
        req.operation = WorldGoal.ADD
        req.body = urdf_body
        req.pose = pose
        req.parent_link = parent_link
        return self._send_goal_and_wait(req)

    def get_control_mode(self) -> ControlModes:
        """
        returns the ControlMode of Giskard
        :return: ControlModes
        """
        rep: TriggerResponse = self._control_mode_srv.call(TriggerRequest())
        return ControlModes[rep.message]

    def dye_group(self, group_name: str, rgba: Tuple[float, float, float, float]) -> DyeGroupResponse:
        """
        Change the color of the ghost for this particular group.
        """
        req = DyeGroupRequest()
        req.group_name = group_name
        req.color.r = rgba[0]
        req.color.g = rgba[1]
        req.color.b = rgba[2]
        req.color.a = rgba[3]
        return self._dye_group_srv(req)

    def get_group_names(self) -> List[str]:
        """
        Returns the names of every group in the world.
        """
        resp: GetGroupNamesResponse = self._get_group_names_srv()
        return resp.group_names

    def get_group_info(self, group_name: str) -> GetGroupInfoResponse:
        """
        Returns the joint state, joint state topic and pose of a group.
        """
        req = GetGroupInfoRequest()
        req.group_name = group_name
        return self._get_group_info_srv.call(req)

    def get_controlled_joints(self, group_name: str) -> List[str]:
        """
        Returns all joints of a group that are flagged as controlled.
        """
        return self.get_group_info(group_name).controlled_joints

    def update_group_pose(self, group_name: str, new_pose: PoseStamped) -> WorldResult:
        """
        Overwrites the pose specified in the joint that connects the two groups.
        :param group_name: Name of the group that will move
        :param new_pose: New pose of the group
        :return: Giskard's reply
        """
        req = WorldGoal()
        req.operation = req.UPDATE_POSE
        req.group_name = group_name
        req.pose = new_pose
        return self._send_goal_and_wait(req)

    def register_group(self, new_group_name: str, root_link_name: Union[str, giskard_msgs.LinkName]) -> WorldResult:
        """
        Register a new group for reference in collision checking. All child links of root_link_name will belong to it.
        :param new_group_name: Name of the new group.
        :param root_link_name: root link of the new group
        :return: WorldResult
        """
        if isinstance(root_link_name, str):
            root_link_name = giskard_msgs.LinkName(root_link_name, '')
        req = WorldGoal()
        req.operation = WorldGoal.REGISTER_GROUP
        req.group_name = new_group_name
        req.parent_link = root_link_name
        return self._send_goal_and_wait(req)


class MotionStatechartNodeWrapper:
    _motion_graph_nodes: Dict[str, MotionStatechartNode]
    _name_prefix = ''
    giskard_wrapper: GiskardWrapper

    def __init__(self, giskard_wrapper: GiskardWrapper):
        self.giskard_wrapper = giskard_wrapper
        self.reset()

    @property
    def robot_name(self) -> str:
        return self.giskard_wrapper.robot_name

    @property
    def motion_graph_nodes(self) -> Dict[str, MotionStatechartNode]:
        return self._motion_graph_nodes

    def reset(self):
        self._motion_graph_nodes = ImmutableDict()

    def _add_motion_statechart_node(self, *,
                                    class_name: str,
                                    name: Optional[str] = None,
                                    start_condition: str = '',
                                    pause_condition: str = '',
                                    end_condition: str = '',
                                    reset_condition: str = '',
                                    **kwargs) -> str:
        """
        Generic function to add a motion goal.
        :param class_name: Name of a class defined in src/giskardpy/goals
        :param name: a unique name for the goal, will use class name by default
        :param start_condition: a logical expression to define the start condition for this monitor. e.g.
                                    not 'monitor1' and ('monitor2' or 'monitor3')
        :param pause_condition: a logical expression. Goal will be on hold if it is True and active otherwise
        :param end_condition: a logical expression. Goal will become inactive when this becomes True.
        :param kwargs: kwargs for __init__ function of class_name
        """
        if name is None:
            name = f'{self._name_prefix}{len(self._motion_graph_nodes)} [{class_name}]'
        motion_goal = MotionStatechartNode()
        motion_goal.name = name
        motion_goal.class_name = class_name
        self._motion_graph_nodes[name] = motion_goal
        motion_goal.kwargs = kwargs_to_json(kwargs)

        self.update_start_condition(node_name=name, condition=start_condition)
        self.update_pause_condition(node_name=name, condition=pause_condition)
        if end_condition is None:  # everything ends themselves by default
            motion_goal.end_condition = name
            self.update_end_condition(node_name=name, condition=name)
        else:
            self.update_end_condition(node_name=name, condition=end_condition)
        self.update_reset_condition(node_name=name, condition=reset_condition)
        return name

    def get_anded_nodes(self, add_nodes_without_end_condition: bool = True) -> str:
        nodes = []
        for node in self.motion_graph_nodes.values():
            if (node.class_name not in get_all_classes_in_package('giskardpy.motion_statechart.monitors',
                                                                  CancelMotion)
                    and (add_nodes_without_end_condition or node.end_condition != '')):
                nodes.append(node.name)
        return ' and '.join(nodes)

    def set_conditions(self, node_name: str,
                       start_condition: str,
                       pause_condition: str,
                       end_condition: str,
                       reset_condition: str):
        self.update_start_condition(node_name, start_condition)
        self.update_pause_condition(node_name, pause_condition)
        self.update_end_condition(node_name, end_condition)
        self.update_reset_condition(node_name, reset_condition)

    def update_start_condition(self, node_name: str, condition: str) -> None:
        self._motion_graph_nodes[node_name].start_condition = condition

    def update_reset_condition(self, node_name: str, condition: str) -> None:
        self._motion_graph_nodes[node_name].reset_condition = condition

    def update_pause_condition(self, node_name: str, condition: str) -> None:
        self._motion_graph_nodes[node_name].pause_condition = condition

    def update_end_condition(self, node_name: str, condition: str) -> None:
        self._motion_graph_nodes[node_name].end_condition = condition


class MotionGoalWrapper(MotionStatechartNodeWrapper):
    _name_prefix = 'G'
    _collision_entries: Dict[Tuple[str, str, str], List[CollisionEntry]]

    def reset(self):
        super().reset()
        self._collision_entries = defaultdict(list)

    def add_motion_goal(self, *,
                        class_name: str,
                        start_condition: str = '',
                        pause_condition: str = '',
                        name: Optional[str] = None,
                        end_condition: str = '',
                        **kwargs) -> str:
        """
        Generic function to add a motion goal.
        :param class_name: Name of a class defined in src/giskardpy/goals
        :param name: a unique name for the goal, will use class name by default
        :param start_condition: a logical expression to define the start condition for this monitor. e.g.
                                    not 'monitor1' and ('monitor2' or 'monitor3')
        :param pause_condition: a logical expression. Goal will be on hold if it is True and active otherwise
        :param end_condition: a logical expression. Goal will become inactive when this becomes True.
        :param kwargs: kwargs for __init__ function of class_name
        """
        return super()._add_motion_statechart_node(class_name=class_name,
                                                   name=name,
                                                   start_condition=start_condition,
                                                   pause_condition=pause_condition,
                                                   end_condition=end_condition,
                                                   **kwargs)

    def add_grasp_bar(self,
                      bar_center: PointStamped,
                      bar_axis: Vector3Stamped,
                      bar_length: float,
                      tip_link: Union[str, giskard_msgs.LinkName],
                      tip_grasp_axis: Vector3Stamped,
                      root_link: Union[str, giskard_msgs.LinkName],
                      name: Optional[str] = None,
                      reference_linear_velocity: Optional[float] = None,
                      reference_angular_velocity: Optional[float] = None,
                      weight: Optional[float] = None,
                      start_condition: str = '',
                      pause_condition: str = '',
                      end_condition: str = '',
                      **kwargs: goal_parameter) -> str:
        """
        Like a CartesianPose but with more freedom.
        tip_link is allowed to be at any point along bar_axis, that is without bar_center +/- bar_length.
        It will align tip_grasp_axis with bar_axis, but allows rotation around it.
        :param root_link: root link of the kinematic chain
        :param tip_link: tip link of the kinematic chain
        :param tip_grasp_axis: axis of tip_link that will be aligned with bar_axis
        :param bar_center: center of the bar to be grasped
        :param bar_axis: alignment of the bar to be grasped
        :param bar_length: length of the bar to be grasped
        :param reference_linear_velocity: m/s
        :param reference_angular_velocity: rad/s
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=GraspBar.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    tip_grasp_axis=tip_grasp_axis,
                                    bar_center=bar_center,
                                    bar_axis=bar_axis,
                                    bar_length=bar_length,
                                    reference_linear_velocity=reference_linear_velocity,
                                    reference_angular_velocity=reference_angular_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_cartesian_pose(self,
                           goal_pose: PoseStamped,
                           tip_link: Union[str, giskard_msgs.LinkName],
                           root_link: Union[str, giskard_msgs.LinkName],
                           name: Optional[str] = None,
                           reference_linear_velocity: Optional[float] = None,
                           reference_angular_velocity: Optional[float] = None,
                           absolute: bool = False,
                           weight: Optional[float] = None,
                           start_condition: str = '',
                           pause_condition: str = '',
                           end_condition: str = '',
                           **kwargs: goal_parameter) -> str:
        """
        This goal will use the kinematic chain between root and tip link to move tip link to the goal pose.
        The max velocities enforce a strict limit, but require a lot of additional constraints, thus making the
        system noticeably slower.
        The reference velocities don't enforce a strict limit, but also don't require any additional constraints.
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain
        :param goal_pose: the goal pose
        :param absolute: if False, the goal pose is reevaluated if start_condition turns True.
        :param reference_linear_velocity: m/s
        :param reference_angular_velocity: rad/s
        :param weight: None = use default weight
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=CartesianPose.__name__,
                                    goal_pose=goal_pose,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    reference_linear_velocity=reference_linear_velocity,
                                    reference_angular_velocity=reference_angular_velocity,
                                    weight=weight,
                                    name=name,
                                    absolute=absolute,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_maximize_manipulability(self,
                                    tip_link: Union[str, giskard_msgs.LinkName],
                                    root_link: Union[str, giskard_msgs.LinkName],
                                    name: Optional[str] = None,
                                    start_condition: str = '',
                                    pause_condition: str = '',
                                    end_condition: str = '',
                                    **kwargs: goal_parameter) -> str:
        """
        This goal maximizes the manipulability of the kinematic chain between root_link and tip_link.
        This chain should only include rotational joint and no linear joints i.e. torso lift joints or odometry joints.
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=MaxManipulability.__name__,
                                    name=name,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_base_arm_weight_scaling(self,
                                    tip_link: Union[str, giskard_msgs.LinkName],
                                    root_link: Union[str, giskard_msgs.LinkName],
                                    tip_goal: PointStamped,
                                    arm_joints: List[str],
                                    base_joints: List[str],
                                    gain: int = 100000,
                                    name: Optional[str] = None,
                                    start_condition: str = '',
                                    pause_condition: str = '',
                                    end_condition: str = '',
                                    **kwargs: goal_parameter) -> str:
        """
        This goals adds weight scaling constraints with the distance between a tip_link and its goal Position as a
        scaling expression. The larger the scaling expression the more is the base movement used to achieve
        all other constraints instead of arm movements. When the expression decreases this relation changes to favor
        arm movements instead of base movements.
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain
        :param tip_goal: the goal position
        :param arm_joints: joints of the arm that should be scaled.
        :param base_joints: joints of the base that should be scaled.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=BaseArmWeightScaling.__name__,
                                    name=name,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    tip_goal=tip_goal,
                                    arm_joints=arm_joints,
                                    base_joints=base_joints,
                                    gain=gain,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_align_planes(self,
                         goal_normal: Vector3Stamped,
                         tip_link: Union[str, giskard_msgs.LinkName],
                         tip_normal: Vector3Stamped,
                         root_link: Union[str, giskard_msgs.LinkName],
                         name: Optional[str] = None,
                         reference_angular_velocity: Optional[float] = None,
                         weight: Optional[float] = None,
                         start_condition: str = '',
                         pause_condition: str = '',
                         end_condition: str = '',
                         **kwargs: goal_parameter) -> str:
        """
        This goal will use the kinematic chain between tip and root to align tip_normal with goal_normal.
        :param goal_normal:
        :param tip_link: tip link of the kinematic chain
        :param tip_normal:
        :param root_link: root link of the kinematic chain
        :param reference_angular_velocity: rad/s
        :param weight:
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=AlignPlanes.__name__,
                                    tip_link=tip_link,
                                    tip_normal=tip_normal,
                                    root_link=root_link,
                                    goal_normal=goal_normal,
                                    max_angular_velocity=reference_angular_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_joint_position(self,
                           goal_state: Dict[str, float],
                           name: Optional[str] = None,
                           weight: Optional[float] = None,
                           max_velocity: Optional[float] = None,
                           threshold: float = 0.01,
                           start_condition: str = '',
                           pause_condition: str = '',
                           end_condition: str = '',
                           **kwargs: goal_parameter) -> str:
        """
        Sets joint position goals for all pairs in goal_state
        :param goal_state: maps joint_name to goal position
        :param weight: None = use default weight
        :param max_velocity: will be applied to all joints
        """
        return self.add_motion_goal(class_name=JointPositionList.__name__,
                                    goal_state=goal_state,
                                    weight=weight,
                                    max_velocity=max_velocity,
                                    name=name,
                                    threshold=threshold,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    # def add_mirror_joint_position(self,
    #                               mapping: Dict[str, str],
    #                               name: Optional[str] = None,
    #                               weight: Optional[float] = None,
    #                               max_velocity: Optional[float] = None,
    #                               start_condition: str = '',
    #                               pause_condition: str = '',
    #                               end_condition: str = '',
    #                               **kwargs: goal_parameter) -> str:
    #     """
    #     Sets joint position goals for all pairs in goal_state
    #     :param goal_state: maps joint_name to goal position
    #     :param weight: None = use default weight
    #     :param max_velocity: will be applied to all joints
    #     """
    #     return self.add_motion_goal(class_name=MirrorJointPosition.__name__,
    #                                 mapping=mapping,
    #                                 weight=weight,
    #                                 max_velocity=max_velocity,
    #                                 name=name,
    #                                 start_condition=start_condition,
    #                                 pause_condition=pause_condition,
    #                                 end_condition=end_condition,
    #                                 **kwargs)

    def add_joint_position_limit(self,
                                 lower_upper_limits: Dict[str, Tuple[float, float]],
                                 name: Optional[str] = None,
                                 weight: Optional[float] = None,
                                 max_velocity: Optional[float] = None,
                                 start_condition: str = '',
                                 pause_condition: str = '',
                                 end_condition: str = '',
                                 **kwargs: goal_parameter) -> str:
        """
        Sets joint position goals for all pairs in goal_state
        :param goal_state: maps joint_name to goal position
        :param weight: None = use default weight
        :param max_velocity: will be applied to all joints
        """
        return self.add_motion_goal(class_name=JointPositionLimitList.__name__,
                                    lower_upper_limits=lower_upper_limits,
                                    weight=weight,
                                    max_velocity=max_velocity,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_justin_torso_limit(self,
                               name: Optional[str] = None,
                               root_link: Union[str, giskard_msgs.LinkName] = 'torso1',
                               tip_link: Union[str, giskard_msgs.LinkName] = 'torso4',
                               forward_distance: float = 0.05,
                               backward_distance: float = 0.14,
                               weight: float = WEIGHT_ABOVE_CA,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '',
                               **kwargs: goal_parameter) -> str:
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=JustinTorsoLimitCart.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    forward_distance=forward_distance,
                                    backward_distance=backward_distance,
                                    name=name,
                                    weight=weight,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_cartesian_position(self,
                               goal_point: PointStamped,
                               tip_link: Union[str, giskard_msgs.LinkName],
                               root_link: Union[str, giskard_msgs.LinkName],
                               name: Optional[str] = None,
                               reference_velocity: Optional[float] = 0.2,
                               weight: Optional[float] = None,
                               absolute: bool = False,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '',
                               **kwargs: goal_parameter) -> str:
        """
        Will use kinematic chain between root_link and tip_link to move tip_link to goal_point.
        :param goal_point:
        :param tip_link: tip link of the kinematic chain
        :param root_link: root link of the kinematic chain
        :param reference_velocity: m/s
        :param weight:
        :param absolute: if False, the goal pose is reevaluated if start_condition turns True.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=CartesianPosition.__name__,
                                    goal_point=goal_point,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    reference_velocity=reference_velocity,
                                    weight=weight,
                                    absolute=absolute,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_cartesian_orientation(self,
                                  goal_orientation: QuaternionStamped,
                                  tip_link: Union[str, giskard_msgs.LinkName],
                                  root_link: Union[str, giskard_msgs.LinkName],
                                  name: Optional[str] = None,
                                  reference_velocity: Optional[float] = None,
                                  weight: Optional[float] = None,
                                  absolute: bool = False,
                                  start_condition: str = '',
                                  pause_condition: str = '',
                                  end_condition: str = '',
                                  **kwargs: goal_parameter) -> str:
        """
        Will use kinematic chain between root_link and tip_link to move tip_link to goal_orientation.
        :param goal_orientation:
        :param tip_link: tip link of kinematic chain
        :param root_link: root link of kinematic chain
        :param tip_group: if tip link is not unique, you can use this to tell Giskard in which group to search.
        :param root_group: if root link is not unique, you can use this to tell Giskard in which group to search.
        :param reference_velocity: rad/s, approx limit
        :param absolute: if False, the goal pose is reevaluated if start_condition turns True.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=CartesianOrientation.__name__,
                                    goal_orientation=goal_orientation,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    reference_velocity=reference_velocity,
                                    weight=weight,
                                    absolute=absolute,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_pointing(self,
                     goal_point: PointStamped,
                     tip_link: Union[str, giskard_msgs.LinkName],
                     pointing_axis: Vector3Stamped,
                     root_link: Union[str, giskard_msgs.LinkName],
                     name: Optional[str] = None,
                     max_velocity: float = 0.3,
                     threshold: float = 0.01,
                     weight: Optional[float] = None,
                     start_condition: str = '',
                     pause_condition: str = '',
                     end_condition: str = '',
                     **kwargs: goal_parameter) -> str:
        """
        Will orient pointing_axis at goal_point.
        :param tip_link: tip link of the kinematic chain.
        :param goal_point: where to point pointing_axis at.
        :param root_link: root link of the kinematic chain.
        :param pointing_axis: the axis of tip_link that will be used for pointing
        :param max_velocity: rad/s
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=Pointing.__name__,
                                    tip_link=tip_link,
                                    goal_point=goal_point,
                                    root_link=root_link,
                                    pointing_axis=pointing_axis,
                                    max_velocity=max_velocity,
                                    threshold=threshold,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_wiggle_insert(self,
                          root_link: giskard_msgs.LinkName,
                          tip_link: giskard_msgs.LinkName,
                          hole_point: PointStamped,
                          name: Optional[str] = None,
                          down_velocity: float = 0.2,
                          noise_translation: float = 0.5,
                          noise_angle: float = 10,
                          random_walk: bool = True,
                          vector_momentum_factor: float = 0.9,
                          angular_momentum_factor: float = 0.9,
                          center_pull_strength_angle: float = 0.1,
                          center_pull_strength_vector: float = 0.25,
                          weight: Optional[float] = None,
                          start_condition: str = '',
                          pause_condition: str = '',
                          end_condition: str = ''):
        """
        Press down while wiggling the end effector.
        :param root_link:
        :param tip_link:
        :param hole_point: Center point of the hole
        :param hole_normal: Vector perpendicular to the hole. default = z-axis of map
        :param threshold: threshold for distance to hole_point to end task
        :param noise_translation: describes how strong the translation wiggle is.
        :param noise_angle: describes how strong the angular wiggle is.
        :param random_walk: determines if random walk or random sample strategy is used
        :param vector_momentum_factor: (only when random_walk=True) Higher value increases influence of momentum,
                                                                    creating smoother but less random translation movement
        :param angular_momentum_factor: (only when random_walk=True) Higher value increases influence of momentum,
                                                                     creating smoother but less random angular movement
        :param center_pull_strength_angle: (only when random_walk=True) Forces angular movement faster back towards
                                                                        starting angle
        :param center_pull_strength_vector: (only when random_walk=True) Forces translation movement faster back towards
                                                                         hole_point
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=WiggleInsert.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    name=name,
                                    down_velocity=down_velocity,
                                    hole_point=hole_point,
                                    noise_translation=noise_translation,
                                    noise_angle=noise_angle,
                                    random_walk=random_walk,
                                    vector_momentum_factor=vector_momentum_factor,
                                    angular_momentum_factor=angular_momentum_factor,
                                    center_pull_strength_angle=center_pull_strength_angle,
                                    center_pull_strength_vector=center_pull_strength_vector,
                                    weight=weight,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def _add_collision_avoidance(self,
                                 collisions: List[CollisionEntry],
                                 start_condition: str = '',
                                 pause_condition: str = '',
                                 end_condition: str = ''):
        key = (start_condition, pause_condition, end_condition)
        self._collision_entries[key].extend(collisions)

    def _add_collision_entries_as_goals(self):
        for (start_condition, pause_condition, end_condition), collision_entries in self._collision_entries.items():
            if (collision_entries[-1].type == CollisionEntry.ALLOW_COLLISION
                    and collision_entries[-1].group1 == CollisionEntry.ALL
                    and collision_entries[-1].group2 == CollisionEntry.ALL):
                continue
            name = 'collision avoidance'
            if start_condition or pause_condition or end_condition:
                name += f'{start_condition}, {pause_condition}, {end_condition}'
            self.add_motion_goal(class_name=CollisionAvoidance.__name__,
                                 name=name,
                                 collision_entries=collision_entries,
                                 start_condition=start_condition,
                                 pause_condition=pause_condition,
                                 end_condition=end_condition)

    def allow_collision(self,
                        group1: str = CollisionEntry.ALL,
                        group2: str = CollisionEntry.ALL,
                        start_condition: str = '',
                        pause_condition: str = '',
                        end_condition: str = ''):
        """
        Tell Giskard to allow collision between group1 and group2. Use CollisionEntry. ALL to allow collision with all
        groups.
        :param group1: name of the first group
        :param group2: name of the second group
        """
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.ALLOW_COLLISION
        collision_entry.group1 = str(group1)
        collision_entry.group2 = str(group2)
        self._add_collision_avoidance(collisions=[collision_entry],
                                      start_condition=start_condition,
                                      pause_condition=pause_condition,
                                      end_condition=end_condition)

    def avoid_collision(self,
                        min_distance: Optional[float] = None,
                        group1: str = CollisionEntry.ALL,
                        group2: str = CollisionEntry.ALL,
                        start_condition: str = '',
                        pause_condition: str = '',
                        end_condition: str = ''):
        """
        Tell Giskard to avoid collision between group1 and group2. Use CollisionEntry. ALL to allow collision with all
        groups.
        :param min_distance: set this to overwrite the default distances
        :param group1: name of the first group
        :param group2: name of the second group
        """
        if min_distance is None:
            min_distance = - 1
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.AVOID_COLLISION
        collision_entry.distance = min_distance
        collision_entry.group1 = group1
        collision_entry.group2 = group2
        self._add_collision_avoidance(collisions=[collision_entry],
                                      start_condition=start_condition,
                                      pause_condition=pause_condition,
                                      end_condition=end_condition)

    def allow_all_collisions(self,
                             start_condition: str = '',
                             pause_condition: str = '',
                             end_condition: str = ''):
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.ALLOW_COLLISION
        self._add_collision_avoidance(collisions=[collision_entry],
                                      start_condition=start_condition,
                                      pause_condition=pause_condition,
                                      end_condition=end_condition)

    def avoid_all_collisions(self,
                             min_distance: Optional[float] = None,
                             start_condition: str = '',
                             pause_condition: str = '',
                             end_condition: str = ''):
        """
        If you don't want to override the distance, don't call this function. Avoid all is the default, if you don't
        add any collision entries.
        :param min_distance: set this to overwrite default distances
        """
        if min_distance is None:
            min_distance = -1
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.AVOID_COLLISION
        collision_entry.distance = min_distance
        self._add_collision_avoidance(collisions=[collision_entry],
                                      start_condition=start_condition,
                                      pause_condition=pause_condition,
                                      end_condition=end_condition)

    def allow_self_collision(self,
                             robot_name: Optional[str] = None,
                             start_condition: str = '',
                             pause_condition: str = '',
                             end_condition: str = ''):
        """
        Allows the collision of the robot with itself for the next goal.
        :param robot_name: if there are multiple robots, specify which one.
        """
        if robot_name is None:
            robot_name = self.robot_name
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.ALLOW_COLLISION
        collision_entry.group1 = robot_name
        collision_entry.group2 = robot_name
        self._add_collision_avoidance(collisions=[collision_entry],
                                      start_condition=start_condition,
                                      pause_condition=pause_condition,
                                      end_condition=end_condition)

    def add_avoid_joint_limits(self,
                               name: Optional[str] = None,
                               percentage: int = 15,
                               joint_list: Optional[List[str]] = None,
                               weight: Optional[float] = None,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '') -> str:
        """
        This goal will push joints away from their position limits. For example if percentage is 15 and the joint
        limits are 0-100, it will push it into the 15-85 range.
        """
        return self.add_motion_goal(class_name=AvoidJointLimits.__name__,
                                    percentage=percentage,
                                    weight=weight,
                                    joint_list=joint_list,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def add_close_container(self,
                            tip_link: Union[str, giskard_msgs.LinkName],
                            environment_link: Union[str, giskard_msgs.LinkName],
                            name: Optional[str] = None,
                            goal_joint_state: Optional[float] = None,
                            weight: Optional[float] = None,
                            start_condition: str = '',
                            pause_condition: str = '',
                            end_condition: str = '') -> str:
        """
        Same as Open, but will use minimum value as default for goal_joint_state
        """
        if isinstance(environment_link, str):
            environment_link = giskard_msgs.LinkName(name=environment_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=Close.__name__,
                                    tip_link=tip_link,
                                    environment_link=environment_link,
                                    goal_joint_state=goal_joint_state,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def add_open_container(self,
                           tip_link: Union[str, giskard_msgs.LinkName],
                           environment_link: Union[str, giskard_msgs.LinkName],
                           name: Optional[str] = None,
                           goal_joint_state: Optional[float] = None,
                           weight: Optional[float] = None,
                           start_condition: str = '',
                           pause_condition: str = '',
                           end_condition: str = '') -> str:
        """
        Open a container in an environment.
        Only works with the environment was added as urdf.
        Assumes that a handle has already been grasped.
        Can only handle containers with 1 dof, e.g. drawers or doors.
        :param tip_link: end effector that is grasping the handle
        :param environment_link: name of the handle that was grasped
        :param goal_joint_state: goal state for the container. default is maximum joint state.
        :param weight:
        """
        if isinstance(environment_link, str):
            environment_link = giskard_msgs.LinkName(name=environment_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=Open.__name__,
                                    tip_link=tip_link,
                                    environment_link=environment_link,
                                    goal_joint_state=goal_joint_state,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def add_align_to_push_door(self,
                               root_link: str,
                               tip_link: str,
                               door_object: str,
                               door_handle: str,
                               tip_gripper_axis: Vector3Stamped,
                               weight: float,
                               name: Optional[str] = None,
                               goal_angle: float = None,
                               tip_group: Optional[str] = None,
                               root_group: Optional[str] = None,
                               intermediate_point_scale: Optional[float] = 1,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '') -> str:
        """
        Aligns the tip_link with the door_object to push it open. Only works if the door object is part of the urdf.
        The door has to be open a little before aligning.
        : param root_link: root link of the kinematic chain
        : param tip_link: end effector
        : param door object: name of the object to be pushed
        : param door_height: height of the door
        : param door_handle: name of the object handle
        : param object_joint_name: name of the joint that rotates
        : param tip_gripper_axis: axis of the tip_link that will be aligned along the door rotation axis
        : param object_rotation_axis: door rotation axis w.r.t root
        """
        return self.add_motion_goal(class_name=AlignToPushDoor.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    door_handle=door_handle,
                                    door_object=door_object,
                                    tip_gripper_axis=tip_gripper_axis,
                                    goal_angle=goal_angle,
                                    tip_group=tip_group,
                                    root_group=root_group,
                                    intermediate_point_scale=intermediate_point_scale,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def add_pre_push_door(self,
                          root_link: str,
                          tip_link: str,
                          door_object: str,
                          door_handle: str,
                          weight: float,
                          threshold: float = 0.01,
                          name: Optional[str] = None,
                          tip_group: Optional[str] = None,
                          root_group: Optional[str] = None,
                          reference_linear_velocity: Optional[float] = None,
                          reference_angular_velocity: Optional[float] = None,
                          start_condition: str = '',
                          pause_condition: str = '',
                          end_condition: str = '') -> str:
        """
        Positions the gripper in contact with the door before pushing to open.
        : param root_link: root link of the kinematic chain
        : param tip_link: end effector
        : param door object: name of the object to be pushed
        : param door_handle: name of the object handle
        : param root_V_object_rotation_axis: door rotation axis w.r.t root
        : param root_V_object_normal: door normal w.r.t root
        """
        return self.add_motion_goal(class_name=PrePushDoor.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    door_object=door_object,
                                    door_handle=door_handle,
                                    tip_group=tip_group,
                                    root_group=root_group,
                                    threshold=threshold,
                                    weight=weight,
                                    name=name,
                                    reference_linear_velocity=reference_linear_velocity,
                                    reference_angular_velocity=reference_angular_velocity,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def add_diff_drive_base(self,
                            goal_pose: PoseStamped,
                            tip_link: Union[str, giskard_msgs.LinkName],
                            root_link: Union[str, giskard_msgs.LinkName],
                            name: Optional[str] = None,
                            reference_linear_velocity: Optional[float] = None,
                            reference_angular_velocity: Optional[float] = None,
                            weight: Optional[float] = None,
                            start_condition: str = '',
                            pause_condition: str = '',
                            end_condition: str = '',
                            **kwargs: goal_parameter) -> str:
        """
        This goal will use the kinematic chain between root and tip link to move tip link into the goal pose.
        It is specifically for differential drives. Will drive towards the goal the following way:
        1. orient to goal
        2. drive to goal position in a straight line
        3. orient to goal orientation
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain
        :param goal_pose: the goal pose
        :param reference_linear_velocity: m/s
        :param reference_angular_velocity: rad/s
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=DiffDriveBaseGoal.__name__,
                                    goal_pose=goal_pose,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    reference_linear_velocity=reference_linear_velocity,
                                    reference_angular_velocity=reference_angular_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_limit_cartesian_velocity(self,
                                     tip_link: Union[str, giskard_msgs.LinkName],
                                     root_link: Union[str, giskard_msgs.LinkName],
                                     name: Optional[str] = None,
                                     max_linear_velocity: float = 0.1,
                                     max_angular_velocity: float = 0.5,
                                     weight: Optional[float] = None,
                                     start_condition: str = '',
                                     pause_condition: str = '',
                                     end_condition: str = '',
                                     **kwargs: goal_parameter) -> str:
        """
        This goal will use put a strict limit on the Cartesian velocity. This will require a lot of constraints, thus
        slowing down the system noticeably.
        :param root_link: root link of the kinematic chain
        :param tip_link: tip link of the kinematic chain
        :param max_linear_velocity: m/s
        :param max_angular_velocity: rad/s
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=CartesianVelocityLimit.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    weight=weight,
                                    max_linear_velocity=max_linear_velocity,
                                    max_angular_velocity=max_angular_velocity,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_real_time_pointing(self,
                               tip_link: Union[str, giskard_msgs.LinkName],
                               pointing_axis: Vector3Stamped,
                               root_link: Union[str, giskard_msgs.LinkName],
                               topic_name: str,
                               name: Optional[str] = None,
                               tip_group: Optional[str] = None,
                               root_group: Optional[str] = None,
                               max_velocity: float = 0.3,
                               weight: Optional[float] = None,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '',
                               **kwargs: goal_parameter) -> str:
        """
        Will orient pointing_axis at goal_point.
        :param tip_link: tip link of the kinematic chain.
        :param topic_name: name of a topic of type PointStamped
        :param root_link: root link of the kinematic chain.
        :param tip_group: if tip_link is not unique, search this group for matches.
        :param root_group: if root_link is not unique, search this group for matches.
        :param pointing_axis: the axis of tip_link that will be used for pointing
        :param max_velocity: rad/s
        """
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)

        return self.add_motion_goal(class_name=RealTimePointing.__name__,
                                    tip_link=tip_link,
                                    tip_group=tip_group,
                                    root_link=root_link,
                                    topic_name=topic_name,
                                    root_group=root_group,
                                    pointing_axis=pointing_axis,
                                    max_velocity=max_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_real_time_cone_pointing(self,
                                    tip_link: Union[str, giskard_msgs.LinkName],
                                    pointing_axis: Vector3Stamped,
                                    cone_theta: float,
                                    root_link: Union[str, giskard_msgs.LinkName],
                                    topic_name: str,
                                    tip_group: Optional[str] = None,
                                    root_group: Optional[str] = None,
                                    max_velocity: float = 0.3,
                                    threshold: float = 0.01,
                                    weight: Optional[float] = None,
                                    start_condition: str = '',
                                    pause_condition: str = '',
                                    end_condition: str = '',
                                    **kwargs: goal_parameter) -> str:
        """
        Will orient pointing_axis at goal_point.
        :param tip_link: tip link of the kinematic chain.
        :param topic_name: name of a topic of type PointStamped
        :param root_link: root link of the kinematic chain.
        :param tip_group: if tip_link is not unique, search this group for matches.
        :param root_group: if root_link is not unique, search this group for matches.
        :param pointing_axis: the axis of tip_link that will be used for pointing
        :param cone_theta: theta angle of viewing cone (angle between right part of cone and pointing axis)
        :param max_velocity: rad/s
        :param threshold:
        :param weight:
        """
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)

        return self.add_motion_goal(class_name=RealTimeConePointing.__name__,
                                    tip_link=tip_link,
                                    tip_group=tip_group,
                                    root_link=root_link,
                                    topic_name=topic_name,
                                    root_group=root_group,
                                    pointing_axis=pointing_axis,
                                    cone_theta=cone_theta,
                                    max_velocity=max_velocity,
                                    threshold=threshold,
                                    weight=weight,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_carry_my_luggage(self,
                             name: Optional[str] = None,
                             tracked_human_position_topic_name: str = '/human_pose',
                             laser_topic_name: str = '/hsrb/base_scan',
                             point_cloud_laser_topic_name: Optional[str] = None,
                             odom_joint_name: str = 'brumbrum',
                             root_link: Optional[str] = None,
                             camera_link: str = 'head_rgbd_sensor_link',
                             distance_to_target_stop_threshold: float = 1,
                             cone_theta: float = 0.1,  # size of FoV cone
                             laser_scan_age_threshold: float = 2,
                             laser_distance_threshold: float = 0.5,
                             laser_distance_threshold_width: float = 0.8,
                             laser_avoidance_angle_cutout: float = np.pi / 4,
                             laser_avoidance_sideways_buffer: float = 0.04,
                             base_orientation_threshold: float = np.pi / 16,
                             tracked_human_position_topic_name_timeout: int = 30,
                             max_rotation_velocity: float = 0.5,
                             max_rotation_velocity_head: float = 2.5,  # head speed
                             max_translation_velocity: float = 0.42,  # base speed
                             traj_tracking_radius: float = 0.4,
                             height_for_camera_target: float = 1,
                             laser_frame_id: str = 'base_range_sensor_link',
                             target_age_threshold: float = 2,
                             target_age_exception_threshold: float = 5,
                             clear_path: bool = False,
                             drive_back: bool = False,
                             enable_laser_avoidance: bool = True,
                             start_condition: str = '',
                             pause_condition: str = '',
                             end_condition: str = '') -> str:
        """
        :param name: name of the goal
        :param tracked_human_position_topic_name: name of the topic where the tracked human is published
        :param laser_topic_name: topic name of the laser scanner
        :param point_cloud_laser_topic_name: topic name of a second laser scanner, e.g. from a point cloud to laser scanner node
        :param odom_joint_name: name of the odom joint
        :param root_link: will use global reference frame
        :param camera_link: link of the camera that will point to the tracked human
        :param distance_to_target_stop_threshold: will pause if closer than this many meter to the target
        :param laser_scan_age_threshold: giskard will complain if scans are older than this many seconds
        :param laser_distance_threshold: this and width are used to crate a stopping zone around the robot.
                                            laser distance draws a circle around the robot and width lines to the left and right.
                                            the stopping zone is the minimum of the two.
        :param laser_distance_threshold_width: see laser_distance_threshold
        :param laser_avoidance_angle_cutout: if something is in the stop zone in front of the robot in +/- this angle range
                                                giskard will pause, otherwise it will try to dodge left or right
        :param laser_avoidance_sideways_buffer: increase this if the robot is shaking too much if something is to its
                                                left and right at the same time.
        :param base_orientation_threshold: giskard will align the base of the robot to the target, this is a +/- buffer to avoid shaking
        :param tracked_human_position_topic_name_timeout: on start up, wait this long for tracking msg to arrive
        :param max_rotation_velocity: how quickly the base can change orientation
        :param max_rotation_velocity_head: how quickly the head rotates
        :param max_translation_velocity: how quickly the base drives
        :param traj_tracking_radius: how close the robots root link will try to stick to the path in meter
        :param height_for_camera_target: target tracking with head will ignore the published height, but use this instead
        :param laser_frame_id: frame_id of the laser scanner
        :param target_age_threshold: will stop looking at the target if the messages are older than this many seconds
        :param target_age_exception_threshold: if there are no messages from the tracked_human_position_topic_name
                                                            topic for this many seconds, cancel
        :param clear_path: clear the saved path. if called repeated will, giskard would just continue the old path if not cleared
        :param drive_back: follow the saved path to drive back
        :param enable_laser_avoidance:
        :param start_condition:
        :param pause_condition:
        :param end_condition:
        """
        return self.add_motion_goal(class_name=CarryMyBullshit.__name__,
                                    name=name,
                                    patrick_topic_name=tracked_human_position_topic_name,
                                    laser_topic_name=laser_topic_name,
                                    point_cloud_laser_topic_name=point_cloud_laser_topic_name,
                                    odom_joint_name=odom_joint_name,
                                    root_link=root_link,
                                    camera_link=camera_link,
                                    distance_to_target_stop_threshold=distance_to_target_stop_threshold,
                                    cone_theta=cone_theta,
                                    laser_scan_age_threshold=laser_scan_age_threshold,
                                    laser_distance_threshold=laser_distance_threshold,
                                    laser_distance_threshold_width=laser_distance_threshold_width,
                                    laser_avoidance_angle_cutout=laser_avoidance_angle_cutout,
                                    laser_avoidance_sideways_buffer=laser_avoidance_sideways_buffer,
                                    base_orientation_threshold=base_orientation_threshold,
                                    wait_for_patrick_timeout=tracked_human_position_topic_name_timeout,
                                    max_rotation_velocity=max_rotation_velocity,
                                    max_rotation_velocity_head=max_rotation_velocity_head,
                                    max_translation_velocity=max_translation_velocity,
                                    traj_tracking_radius=traj_tracking_radius,
                                    height_for_camera_target=height_for_camera_target,
                                    laser_frame_id=laser_frame_id,
                                    target_age_threshold=target_age_threshold,
                                    target_age_exception_threshold=target_age_exception_threshold,
                                    clear_path=clear_path,
                                    drive_back=drive_back,
                                    enable_laser_avoidance=enable_laser_avoidance,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def set_seed_configuration(self,
                               seed_configuration: Dict[str, float],
                               name: Optional[str] = None,
                               group_name: Optional[str] = None):
        """
        Only meant for use with projection. Changes the world state to seed_configuration before starting planning,
        without having to plan a motion to it like with add_joint_position
        """
        raise DeprecationWarning('please use monitors.set_seed_configuration instead')

    def set_seed_odometry(self,
                          base_pose: PoseStamped,
                          name: Optional[str] = None,
                          group_name: Optional[str] = None):
        """
        Only meant for use with projection. Overwrites the odometry transform with base_pose.
        """
        raise DeprecationWarning('please use monitors.add_set_seed_odometry instead')

    def add_cartesian_pose_straight(self,
                                    goal_pose: PoseStamped,
                                    tip_link: Union[str, giskard_msgs.LinkName],
                                    root_link: Union[str, giskard_msgs.LinkName],
                                    name: Optional[str] = None,
                                    reference_linear_velocity: Optional[float] = None,
                                    reference_angular_velocity: Optional[float] = None,
                                    weight: Optional[float] = None,
                                    absolute: bool = False,
                                    start_condition: str = '',
                                    pause_condition: str = '',
                                    end_condition: str = '',
                                    **kwargs: goal_parameter) -> str:
        """
        This goal will use the kinematic chain between root and tip link to move tip link into the goal pose.
        The max velocities enforce a strict limit, but require a lot of additional constraints, thus making the
        system noticeably slower.
        The reference velocities don't enforce a strict limit, but also don't require any additional constraints.
        In contrast to set_cart_goal, this tries to move the tip_link in a straight line to the goal_point.
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain
        :param goal_pose: the goal pose
        :param reference_linear_velocity: m/s
        :param reference_angular_velocity: rad/s
        :param absolute: if False, the goal pose is reevaluated if start_condition turns True.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=CartesianPoseStraight.__name__,
                                    goal_pose=goal_pose,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    weight=weight,
                                    reference_linear_velocity=reference_linear_velocity,
                                    reference_angular_velocity=reference_angular_velocity,
                                    name=name,
                                    absolute=absolute,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_cartesian_position_straight(self,
                                        goal_point: PointStamped,
                                        tip_link: Union[str, giskard_msgs.LinkName],
                                        root_link: Union[str, giskard_msgs.LinkName],
                                        name: Optional[str] = None,
                                        reference_velocity: float = None,
                                        weight: Optional[float] = None,
                                        absolute: bool = False,
                                        start_condition: str = '',
                                        pause_condition: str = '',
                                        end_condition: str = '',
                                        **kwargs: goal_parameter) -> str:
        """
        Same as set_translation_goal, but will try to move in a straight line.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=CartesianPositionStraight.__name__,
                                    goal_point=goal_point,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    reference_velocity=reference_velocity,
                                    weight=weight,
                                    name=name,
                                    absolute=absolute,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_align_perpendicular(self,
                                reference_normal: Vector3Stamped,
                                tip_link: Union[str, giskard_msgs.LinkName],
                                tip_normal: Vector3Stamped,
                                root_link: Union[str, giskard_msgs.LinkName],
                                name: Optional[str] = None,
                                reference_velocity: Optional[float] = None,
                                weight: Optional[float] = None,
                                start_condition: str = '',
                                pause_condition: str = '',
                                end_condition: str = '',
                                **kwargs: goal_parameter) -> str:
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=AlignPerpendicular.__name__,
                                    tip_normal=tip_normal,
                                    reference_normal=reference_normal,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    max_vel=reference_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_height(self,
                   reference_point: PointStamped,
                   tip_point: PointStamped,
                   tip_link: Union[str, giskard_msgs.LinkName],
                   root_link: Union[str, giskard_msgs.LinkName],
                   lower_limit: float,
                   upper_limit: float,
                   name: Optional[str] = None,
                   reference_velocity: Optional[float] = None,
                   weight: Optional[float] = None,
                   start_condition: str = '',
                   pause_condition: str = '',
                   end_condition: str = '',
                   **kwargs: goal_parameter) -> str:
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=HeightGoal.__name__,
                                    tip_point=tip_point,
                                    reference_point=reference_point,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    lower_limit=lower_limit,
                                    upper_limit=upper_limit,
                                    max_vel=reference_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_distance(self,
                     reference_point: PointStamped,
                     tip_point: PointStamped,
                     tip_link: Union[str, giskard_msgs.LinkName],
                     root_link: Union[str, giskard_msgs.LinkName],
                     lower_limit: float,
                     upper_limit: float,
                     name: Optional[str] = None,
                     reference_velocity: Optional[float] = None,
                     weight: Optional[float] = None,
                     start_condition: str = '',
                     pause_condition: str = '',
                     end_condition: str = '',
                     **kwargs: goal_parameter) -> str:
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=DistanceGoal.__name__,
                                    tip_point=tip_point,
                                    reference_point=reference_point,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    lower_limit=lower_limit,
                                    upper_limit=upper_limit,
                                    max_vel=reference_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_angle(self,
                  reference_vector: Vector3Stamped,
                  tip_link: Union[str, giskard_msgs.LinkName],
                  tip_vector: Vector3Stamped,
                  root_link: Union[str, giskard_msgs.LinkName],
                  lower_angle: float,
                  upper_angle: float,
                  name: Optional[str] = None,
                  reference_velocity: Optional[float] = None,
                  weight: Optional[float] = None,
                  start_condition: str = '',
                  pause_condition: str = '',
                  end_condition: str = '',
                  **kwargs: goal_parameter) -> str:
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=AngleGoal.__name__,
                                    tip_vector=tip_vector,
                                    reference_vector=reference_vector,
                                    tip_link=tip_link,
                                    root_link=root_link,
                                    lower_angle=lower_angle,
                                    upper_angle=upper_angle,
                                    max_vel=reference_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    # SuTuRo Goals Interface Start

    def add_grasp_bar_offset(self,
                             bar_center: PointStamped,
                             bar_axis: Vector3Stamped,
                             bar_length: float,
                             tip_link: Union[str, giskard_msgs.LinkName],
                             tip_grasp_axis: Vector3Stamped,
                             root_link: Union[str, giskard_msgs.LinkName],
                             grasp_axis_offset: Vector3Stamped,
                             handle_link: Union[str, giskard_msgs.LinkName],
                             reference_linear_velocity: Optional[float] = None,
                             reference_angular_velocity: Optional[float] = None,
                             weight: Optional[float] = None,
                             name: Optional[str] = None,
                             start_condition: str = '',
                             pause_condition: str = '',
                             end_condition: str = '',
                             **kwargs: goal_parameter) -> str:
        """
        Like a CartesianPose but with more freedom.
        tip_link is allowed to be at any point along bar_axis, that is without bar_center +/- bar_length.
        It will align tip_grasp_axis with bar_axis, but allows rotation around it.
        :param root_link: root link of the kinematic chain
        :param tip_link: tip link of the kinematic chain
        :param tip_grasp_axis: axis of tip_link that will be aligned with bar_axis
        :param bar_center: center of the bar to be grasped
        :param bar_axis: alignment of the bar to be grasped
        :param grasp_axis_offset: offset of the grasp axis
        :param bar_length: length of the bar to be grasped
        :param root_group: if root_link is not unique, search in this group for matches
        :param tip_group: if tip_link is not unique, search in this group for matches
        :param reference_linear_velocity: m/s
        :param reference_angular_velocity: rad/s
        :param weight:
        :param name:
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=GraspBarOffset.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    tip_grasp_axis=tip_grasp_axis,
                                    bar_center=bar_center,
                                    bar_axis=bar_axis,
                                    bar_length=bar_length,
                                    grasp_axis_offset=grasp_axis_offset,
                                    handle_link=handle_link,
                                    reference_linear_velocity=reference_linear_velocity,
                                    reference_angular_velocity=reference_angular_velocity,
                                    weight=weight,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)

    def add_grasp_with_ft_sensor(self,
                                 root_link: Union[str, giskard_msgs.LinkName],
                                 tip_link: Union[str, giskard_msgs.LinkName],
                                 handle_name: Union[str, giskard_msgs.LinkName],
                                 tip_grasp_axis: Vector3Stamped,
                                 bar_axis: Vector3Stamped,
                                 tip_retract: PointStamped,
                                 handle_align_axis: Vector3Stamped,
                                 tip_align_axis: Vector3Stamped,
                                 grasp_axis_offset: Vector3Stamped,
                                 pre_grasp_axis_offset: Vector3Stamped,
                                 hinge_joint: Union[str, giskard_msgs.LinkName],
                                 bar_length: float = 0.01,
                                 timeout: float = 10,
                                 ft_topic: str = '/filtered_raw/diff',
                                 ft_grasp_ref_speed: float = 1,
                                 camera_link: Optional[Union[str, giskard_msgs.LinkName]] = None,
                                 tip_push: Optional[PointStamped] = None,
                                 handle_correction_offset: Optional[PointStamped] = None,
                                 name: str = None,
                                 start_condition: str = '',
                                 pause_condition: str = '',
                                 end_condition: str = '', ) -> str:
        """
        Complex grasping motion using the ForceTorqueMonitor.

        :param root_link: root-link of the kinematic chain.
        :param tip_link: tip-link of the kinematic chain.
        :param handle_name: LinkName of the environment-link that is to be grasped.
        :param tip_grasp_axis: axis of tip-link that is to be aligned with bar axis.
        :param bar_axis: axis of the handle along which the handlebar is.
        :param tip_push:
        :param tip_retract: distance the tip will retract after force-torque-threshold is reached.
        :param handle_align_axis: axis of the handle which will be aligned with the grasp_axis while grasping.
        :param tip_align_axis: axis of the tip_link along which is grasped. e.g. z-axis of the hsrb-gripper.
        :param grasp_axis_offset: offset for grasping, such that force-torque sensor can be triggered.
        :param pre_grasp_axis_offset: offset for pre-grasp-pose, from which the ft-grasping is started.
        :param hinge_joint: hinge joint to be locked in place while grasping.
        :param bar_length: length of the handlebar.
        :param timeout: duration after which the ft-grasping is cancelled.
        :param ft_topic: topic of the sensor-data for the ft-monitor.
        :param ft_grasp_ref_speed: reference-speed-modifier for the ft-grasping motion.
        :param camera_link: Link-name of camera_link when using camera offset via visual servoing
        :param handle_correction_offset: Offset after visual servoing to adjust for middle of handle
        :param name: Name of the Goal
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        if isinstance(handle_name, str):
            handle_name = giskard_msgs.LinkName(name=handle_name)
        if isinstance(hinge_joint, str):
            hinge_joint = giskard_msgs.LinkName(name=hinge_joint)
        if isinstance(camera_link, str):
            camera_link = giskard_msgs.LinkName(name=camera_link)

        return self.add_motion_goal(class_name=GraspWithForceTorqueGoal.__name__,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    handle_name=handle_name,
                                    tip_grasp_axis=tip_grasp_axis,
                                    bar_axis=bar_axis,
                                    tip_push=tip_push,
                                    tip_retract=tip_retract,
                                    handle_align_axis=handle_align_axis,
                                    tip_align_axis=tip_align_axis,
                                    grasp_axis_offset=grasp_axis_offset,
                                    pre_grasp_axis_offset=pre_grasp_axis_offset,
                                    hinge_joint=hinge_joint,
                                    bar_length=bar_length,
                                    timeout=timeout,
                                    ft_topic=ft_topic,
                                    ft_grasp_ref_speed=ft_grasp_ref_speed,
                                    camera_link=camera_link,
                                    handle_correction_offset=handle_correction_offset,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def hsrb_open_door_goal(self,
                            handle_name: Union[str, giskard_msgs.LinkName],
                            tip_link: Union[str, giskard_msgs.LinkName] = giskard_msgs.LinkName(
                                name='hand_gripper_tool_frame'),
                            name: str = 'HSRB_open_door',
                            handle_limit: Optional[float] = (np.pi / 6),
                            hinge_limit: Optional[float] = -(np.pi / 4),
                            start_condition: str = '',
                            pause_condition: str = '',
                            end_condition: str = '') -> str:
        """
        HSRB specific open door goal wrapper

        :param handle_name: Link of the door handle
        :param tip_link: Link that's grasping the door handle
        :param name: name of the goal for distinction between same goals
        :param handle_limit: Limits the handle opening to given value
        :param hinge_limit: Limits the hinge opening to given value
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        return self.add_open_door_goal(tip_link=tip_link,
                                       handle_name=handle_name,
                                       name=name,
                                       handle_limit=handle_limit,
                                       hinge_limit=hinge_limit,
                                       start_condition=start_condition,
                                       pause_condition=pause_condition,
                                       end_condition=end_condition)

    def hsrb_door_handle_grasp(self,
                               handle_name: str,
                               handle_bar_length: float = 0,
                               tip_link: str = 'hand_gripper_tool_frame',
                               root_link: str = 'map',
                               grasp_axis_offset: Optional[Vector3Stamped] = None,
                               bar_axis_v: Optional[Vector3Stamped] = None,
                               tip_grasp_axis_v: Optional[Vector3Stamped] = None,
                               name: Optional[str] = None,
                               ref_speed: Optional[float] = 1,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '') -> str:
        """
        HSRB specific set_grasp_bar_goal, that only needs handle_name of the door_handle

        :param handle_name: URDF link that represents the door handle
        :param handle_bar_length: length of the door handle
        :param tip_link: robot link, that grasps the handle
        :param root_link: root link of the kinematic chain
        :param grasp_axis_offset: Offset for end-effector
        :param bar_axis_v: Vector for changing the orientation of the door handle
        :param tip_grasp_axis_v: Vector for the orientation of the tip grasp link
        :param name: Name of the goal
        :param ref_speed: Reference speed for linear and angular velocities, used to slow down movement
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_name
        if bar_axis_v is None:
            bar_axis.vector = Vector3(0, 1, 0)
        else:
            bar_axis.vector = bar_axis_v

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip_link
        if tip_grasp_axis_v is None:
            tip_grasp_axis.vector = Vector3(1, 0, 0)
        else:
            tip_grasp_axis.vector = tip_grasp_axis_v

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_name

        if grasp_axis_offset is None:
            return self.add_grasp_bar(name=name,
                                      root_link=root_link,
                                      tip_link=tip_link,
                                      tip_grasp_axis=tip_grasp_axis,
                                      bar_center=bar_center,
                                      bar_axis=bar_axis,
                                      bar_length=handle_bar_length,
                                      reference_linear_velocity=0.1 * ref_speed,
                                      reference_angular_velocity=0.5 * ref_speed,
                                      start_condition=start_condition,
                                      pause_condition=pause_condition,
                                      end_condition=end_condition)
        else:
            return self.add_grasp_bar_offset(name=name,
                                             root_link=root_link,
                                             tip_link=tip_link,
                                             tip_grasp_axis=tip_grasp_axis,
                                             bar_center=bar_center,
                                             bar_axis=bar_axis,
                                             bar_length=handle_bar_length,
                                             handle_link=handle_name,
                                             reference_linear_velocity=0.1 * ref_speed,
                                             reference_angular_velocity=0.5 * ref_speed,
                                             grasp_axis_offset=grasp_axis_offset,
                                             start_condition=start_condition,
                                             pause_condition=pause_condition,
                                             end_condition=end_condition)

    def add_move_around_hinge(self,
                              handle_name: Union[str, giskard_msgs.LinkName],
                              tip_gripper_axis: Vector3Stamped = None,
                              root_link: Union[str, giskard_msgs.LinkName] = 'map',
                              tip_link: Union[str, giskard_msgs.LinkName] = 'hand_gripper_tool_frame',
                              goal_angle: float = None,
                              name: Optional[str] = None,
                              multipliers: Optional[List[Tuple[float, float, str]]] = None,
                              offset: Optional[Vector3Stamped] = None,
                              align_gripper: Union[int, MoveAroundHingeAlign] = MoveAroundHingeAlign.LAST,
                              start_condition: str = '',
                              pause_condition: str = '',
                              end_condition: str = '') -> str:
        """
        Move around hinge to given handle_name, based on distance between hinge and handle

        :param handle_name: name of the handle
        :param tip_gripper_axis: tip axis of gripper to point towards handle
        :param tip_link: robot link, that grasps the handle
        :param root_link: root link of the kinematic chain
        :param goal_angle: Angle that the hinge should have at start of goal
        :param name: Name of the goal
        :param multipliers: Multipliers for distance and angle between handle and joint,
         to control where move around points are and their goal_names
        :param offset: Offset for move around points, best used for height offset when handling doors
        :param align_gripper: Decides if gripper should try to align towards handle at each point or only last
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        if isinstance(handle_name, str):
            handle_name = giskard_msgs.LinkName(name=handle_name)
        if isinstance(align_gripper, int):
            try:
                align_gripper = MoveAroundHingeAlign(align_gripper)
            except ValueError:
                raise ValueError("Invalid MoveAroundHingeAlign value")

        return self.add_motion_goal(class_name=MoveAroundHinge.__name__,
                                    handle_name=handle_name,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    goal_angle=goal_angle,
                                    name=name,
                                    tip_gripper_axis=tip_gripper_axis,
                                    multipliers=multipliers,
                                    offset=offset,
                                    align_gripper=align_gripper,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def hsrb_dishwasher_door_around(self,
                                    handle_name: str,
                                    tip_gripper_axis: Vector3Stamped = None,
                                    root_link: str = 'map',
                                    tip_link: str = 'hand_gripper_tool_frame',
                                    goal_angle: float = None,
                                    name: Optional[str] = None,
                                    start_condition: str = '',
                                    pause_condition: str = '',
                                    end_condition: str = '') -> str:
        """
        HSRB specific avoid dishwasher door goal

        :param handle_name: name of the dishwasher handle
        :param tip_gripper_axis: tip axis of gripper to point towards dishwasher door
        :param tip_link: robot link, that grasps the handle
        :param root_link: root link of the kinematic chain
        :param goal_angle: Angle that the dishwasher door should have at start of goal
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        return self.add_move_around_hinge(handle_name=handle_name,
                                          root_link=root_link,
                                          tip_link=tip_link,
                                          goal_angle=goal_angle,
                                          name=name,
                                          tip_gripper_axis=tip_gripper_axis,
                                          start_condition=start_condition,
                                          pause_condition=pause_condition,
                                          end_condition=end_condition)

    def hsrb_align_to_push_door_goal(self,
                                     handle_name: str,
                                     hinge_frame_id: str,
                                     tip_link: str = 'hand_gripper_tool_frame',
                                     root_link: str = 'map',
                                     weight: float = WEIGHT_ABOVE_CA):
        """
        HSRB specific push door open goal of dishwasher

        :param handle_name: name of the door handle
        :param hinge_frame_id: Frame id of the door hinge
        :param tip_link: robot link, that grasps the handle
        :param root_link: root link of the kinematic chain
        :param weight: Weight of the goal compared to Collision Avoidance
        """

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip_link
        tip_grasp_axis.vector.y = 1

        return self.add_align_to_push_door(root_link=root_link,
                                           tip_link=tip_link,
                                           door_handle=handle_name,
                                           door_object=hinge_frame_id,
                                           tip_gripper_axis=tip_grasp_axis,
                                           weight=weight,
                                           intermediate_point_scale=0.95)

    def hsrb_pre_push_door_goal(self,
                                handle_name: str,
                                hinge_frame_id: str,
                                root_link: str = 'map',
                                tip_link: str = 'hand_gripper_tool_frame',
                                weight: float = WEIGHT_ABOVE_CA):
        """
        HSRB specific pre push door open goal of dishwasher

        :param handle_name: name of the door handle
        :param hinge_frame_id: Frame id of the door hinge
        :param tip_link: robot link, that grasps the handle
        :param root_link: root link of the kinematic chain
        :param weight: Weight of the goal compared to Collision Avoidance
        """

        return self.add_pre_push_door(root_link=root_link,
                                      tip_link=tip_link,
                                      door_handle=handle_name,
                                      weight=weight,
                                      door_object=hinge_frame_id)

    # TODO: change object_size Vector3Stamped instead
    def add_reaching(self,
                     grasp: str,
                     align: str,
                     object_name: str,
                     object_shape: str,
                     goal_pose: Optional[PoseStamped] = None,
                     object_size: Optional[Vector3] = None,
                     root_link: str = 'map',
                     tip_link: str = 'hand_palm_link',
                     velocity: float = 0.2):
        """
        :param grasp: direction to grasp from, directions are: front, top, right, left, below
        :param align: the frame that the wrist frame aligns with, will be ignored if left empty
        :param object_name: name of object that should be reached
        :param object_shape: shape of the object (current options are cylinder, sphere and rectangle)
        :param goal_pose: position of the goal that should be reached
        :param object_size: size of the object as a Vector3 (in meters)
        :param root_link: the root link, usually map
        :param tip_link: the tip link, normally hand_palm_link
        :param velocity: velocity of executed movement
        """

        return self.add_motion_goal(class_name=Reaching.__name__,
                                    grasp=grasp,
                                    align=align,
                                    object_name=object_name,
                                    object_shape=object_shape,
                                    goal_pose=goal_pose,
                                    object_size=object_size,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    velocity=velocity)

    def add_placing(self,
                    context,
                    goal_pose: PoseStamped,
                    tip_link: str = 'hand_palm_link',
                    velocity: float = 0.02):

        return self.add_motion_goal(class_name=Placing.__name__,
                                    context=context,
                                    goal_pose=goal_pose,
                                    tip_link=tip_link,
                                    velocity=velocity)

    def add_vertical_motion(self,
                            action: str,
                            distance: float = 0.02,
                            root_link: str = 'base_link',
                            tip_link: str = 'hand_palm_link'):

        return self.add_motion_goal(class_name=VerticalMotion.__name__,
                                    action=action,
                                    distance=distance,
                                    root_link=root_link,
                                    tip_link=tip_link)

    def add_retract(self,
                    object_name: str,
                    distance: float = 0.1,
                    reference_frame: str = 'base_link',
                    root_link: str = 'map',
                    tip_link: str = 'base_link',
                    velocity: float = 0.2):

        return self.add_motion_goal(class_name=Retracting.__name__,
                                    object_name=object_name,
                                    distance=distance,
                                    reference_frame=reference_frame,
                                    root_link=root_link,
                                    tip_link=tip_link,
                                    velocity=velocity)

    def add_align_height(self,
                         object_name: str,
                         goal_pose: PoseStamped,
                         object_height: float,
                         from_above: bool = False,
                         root_link: str = 'map',
                         tip_link: str = 'hand_gripper_tool_frame'):

        return self.add_motion_goal(class_name=AlignHeight.__name__,
                                    from_above=from_above,
                                    object_name=object_name,
                                    goal_pose=goal_pose,
                                    object_height=object_height,
                                    root_link=root_link,
                                    tip_link=tip_link)

    def add_test_goal(self,
                      goal_name: str,
                      **kwargs):

        return self.add_motion_goal(class_name=goal_name,
                                    **kwargs)

    def add_take_pose(self,
                      pose_keyword: str,
                      start_condition: str = '',
                      pause_condition: str = '',
                      end_condition: str = ''):
        """
        Goal for Taking a given keyword joint pose

        :param pose_keyword: Keyword of joint pose
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """

        return self.add_motion_goal(class_name=TakePose.__name__,
                                    pose_keyword=pose_keyword,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def add_tilting(self,
                    tilt_direction: Optional[str] = None,
                    tilt_angle: Optional[float] = None,
                    tip_link: str = 'wrist_roll_joint',
                    ):

        return self.add_motion_goal(class_name=Tilting.__name__,
                                    direction=tilt_direction,
                                    angle=tilt_angle,
                                    tip_link=tip_link)

    def add_joint_rotation_continuous(self,
                                      joint_name: str,
                                      joint_center: float,
                                      joint_range: float,
                                      trajectory_length: float = 20,
                                      target_speed: float = 1,
                                      period_length: float = 1.0):

        return self.add_motion_goal(class_name=JointRotationGoalContinuous.__name__,
                                    joint_name=joint_name,
                                    joint_center=joint_center,
                                    joint_range=joint_range,
                                    trajectory_length=trajectory_length,
                                    target_speed=target_speed,
                                    period_length=period_length)

    def add_mixing(self,
                   mixing_time=20,
                   weight: float = WEIGHT_ABOVE_CA):

        return self.add_motion_goal(class_name=Mixing.__name__,
                                    mixing_time=mixing_time,
                                    weight=weight)

    def add_open_environment(self,
                             tip_link: str,
                             environment_link: str,
                             tip_group: Optional[str] = None,
                             environment_group: Optional[str] = None,
                             goal_joint_state: Optional[float] = None,
                             weight: float = WEIGHT_ABOVE_CA):
        """
        Adds OpenGoal to motion goal execution plan

        :param tip_link: Link that is grasping the handle
        :param environment_link: Link that is grasped
        :param goal_joint_state: State of the joint that the opening motion tries to reach
        :param weight: Weight of the goal compared to Collision Avoidance
        """

        return self.add_motion_goal(class_name=Open.__name__,
                                    tip_link=tip_link,
                                    environment_link=environment_link,
                                    goal_joint_state=goal_joint_state,
                                    weight=weight)

    def add_open_door_goal(self,
                           tip_link: Union[str, giskard_msgs.LinkName],
                           handle_name: Union[str, giskard_msgs.LinkName],
                           name: str = None,
                           handle_limit: Optional[float] = None,
                           hinge_limit: Optional[float] = None,
                           start_condition: str = '',
                           pause_condition: str = '',
                           end_condition: str = ''):
        """
        Adds OpenDoorGoal to motion goal execution plan

        :param tip_link: Link that is grasping the door handle
        :param handle_name: Link of the door handle of the door that is to be opened
        :param name: Name of the Goal for distinction between similar goals
        :param handle_limit: Limits the handle opening to given value
        :param hinge_limit: Limits the hinge opening to given value
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        if isinstance(handle_name, str):
            handle_name = giskard_msgs.LinkName(name=handle_name)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_motion_goal(class_name=OpenDoorGoal.__name__,
                                    tip_link=tip_link,
                                    handle_name=handle_name,
                                    name=name,
                                    handle_limit=handle_limit,
                                    hinge_limit=hinge_limit,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition)

    def continuous_pointing_head(self):
        """
        Uses real_time_pointing for continuous tracking of a human_pose.
        """
        tip_V_pointing_axis: Vector3Stamped = Vector3Stamped()
        tip_V_pointing_axis.header.frame_id = 'head_center_camera_frame'
        tip_V_pointing_axis.vector.z = 1

        return self.add_real_time_pointing(root_link='map',
                                           tip_link='head_center_camera_frame',
                                           topic_name='human_pose',
                                           pointing_axis=tip_V_pointing_axis)

    def add_joint_position_stop(self,
                                goal_state: Dict[str, float],
                                weight: Optional[float] = None,
                                max_velocity: Optional[float] = None,
                                name: Optional[str] = None,
                                start_condition: str = '',
                                pause_condition: str = '',
                                end_condition: str = '',
                                **kwargs: goal_parameter):
        """
        Sets joint position goals for all pairs in goal_state
        :param goal_state: maps joint_name to goal position
        :param weight: None = use default weight
        :param max_velocity: will be applied to all joints
        :param name: name of the goal, used to distinguish same type of goal
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        return self.add_motion_goal(class_name=JointPositionListStop.__name__,
                                    goal_state=goal_state,
                                    weight=weight,
                                    max_velocity=max_velocity,
                                    name=name,
                                    start_condition=start_condition,
                                    pause_condition=pause_condition,
                                    end_condition=end_condition,
                                    **kwargs)


class MonitorWrapper(MotionStatechartNodeWrapper):
    _name_prefix = 'M'

    def reset(self):
        super().reset()
        self.max_trajectory_length_set = False

    def add_monitor(self, *,
                    class_name: str,
                    name: Optional[str] = None,
                    start_condition: str = '',
                    pause_condition: str = '',
                    end_condition: str = '',
                    reset_condition: str = '',
                    **kwargs) -> str:
        """
        Generic function to add a monitor.
        :param class_name: Name of a class defined in src/giskardpy/monitors
        :param name: a unique name for the goal, will use class name by default
        :param start_condition: a logical expression to define the start condition for this monitor. e.g.
                                    not 'monitor1' and ('monitor2' or 'monitor3')
        :param pause_condition: a logical expression to define the hold condition for this monitor.
        :param end_condition: a logical expression to define the end condition for this monitor.
        :param kwargs: kwargs for __init__ function of class_name
        :return: the name of the monitor with added quotes to be used in logical expressions for conditions.
        """
        return super()._add_motion_statechart_node(class_name=class_name,
                                                   name=name,
                                                   start_condition=start_condition,
                                                   pause_condition=pause_condition,
                                                   end_condition=end_condition,
                                                   reset_condition=reset_condition,
                                                   **kwargs)

    def add_local_minimum_reached(self,
                                  name: Optional[str] = None,
                                  start_condition: str = '',
                                  pause_condition: str = '',
                                  end_condition: str = '',
                                  reset_condition: str = '') -> str:
        """
        True if the world is currently in a local minimum.
        """
        return self.add_monitor(class_name=LocalMinimumReached.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_time_above(self,
                       threshold: float,
                       name: Optional[str] = None,
                       start_condition: str = '',
                       pause_condition: str = '',
                       end_condition: str = '',
                       reset_condition: str = '') -> str:
        """
        True if the length of the trajectory is above threshold
        """
        return self.add_monitor(class_name=TimeAbove.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                threshold=threshold,
                                reset_condition=reset_condition)

    def add_joint_position(self,
                           goal_state: Dict[str, float],
                           name: Optional[str] = None,
                           threshold: float = 0.01,
                           start_condition: str = '',
                           pause_condition: str = '',
                           end_condition: str = '',
                           reset_condition: str = '') -> str:
        """
        True if all joints in goal_state are closer than threshold to their respective value.
        """
        return self.add_monitor(class_name=JointGoalReached.__name__,
                                name=name,
                                goal_state=goal_state,
                                threshold=threshold,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_joint_position_above(self,
                                 joint_name: Union[str, giskard_msgs.LinkName],
                                 threshold: float,
                                 name: Optional[str] = None,
                                 start_condition: str = '',
                                 pause_condition: str = '',
                                 end_condition: str = '',
                                 reset_condition: str = '') -> str:
        """
        True if `joint_name` is above `threshold`.
        """
        if isinstance(joint_name, str):
            joint_name = giskard_msgs.LinkName(name=joint_name)
        return self.add_monitor(class_name=JointPositionAbove.__name__,
                                name=name,
                                joint_name=joint_name,
                                threshold=threshold,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_cartesian_pose(self,
                           root_link: Union[str, giskard_msgs.LinkName],
                           tip_link: Union[str, giskard_msgs.LinkName],
                           goal_pose: PoseStamped,
                           name: Optional[str] = None,
                           position_threshold: float = 0.01,
                           orientation_threshold: float = 0.01,
                           absolute: bool = False,
                           start_condition: str = '',
                           pause_condition: str = '',
                           end_condition: str = '',
                           reset_condition: str = '') -> str:
        """
        True if tip_link is closer than the thresholds to goal_pose.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=PoseReached.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                goal_pose=goal_pose,
                                absolute=absolute,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                position_threshold=position_threshold,
                                orientation_threshold=orientation_threshold)

    def add_cartesian_position(self,
                               root_link: Union[str, giskard_msgs.LinkName],
                               tip_link: Union[str, giskard_msgs.LinkName],
                               goal_point: PointStamped,
                               name: Optional[str] = None,
                               threshold: float = 0.01,
                               absolute: bool = False,
                               start_condition: str = '',
                               pause_condition: str = '',
                               end_condition: str = '',
                               reset_condition: str = '') -> str:
        """
        True if tip_link is closer than threshold to goal_point.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=PositionReached.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                goal_point=goal_point,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                absolute=absolute,
                                threshold=threshold)

    def add_distance_to_line(self,
                             root_link: Union[str, giskard_msgs.LinkName],
                             tip_link: Union[str, giskard_msgs.LinkName],
                             center_point: PointStamped,
                             line_axis: Vector3Stamped,
                             line_length: float,
                             name: Optional[str] = None,
                             start_condition: str = '',
                             pause_condition: str = '',
                             end_condition: str = '',
                             reset_condition: str = '',
                             threshold: float = 0.01) -> str:
        """
        True if tip_link is closer than threshold to the line defined by center_point, line_axis and line_length.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=DistanceToLine.__name__,
                                name=name,
                                center_point=center_point,
                                line_axis=line_axis,
                                line_length=line_length,
                                root_link=root_link,
                                tip_link=tip_link,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                threshold=threshold)

    def add_cartesian_orientation(self,
                                  root_link: Union[str, giskard_msgs.LinkName],
                                  tip_link: Union[str, giskard_msgs.LinkName],
                                  goal_orientation: QuaternionStamped,
                                  name: Optional[str] = None,
                                  threshold: float = 0.01,
                                  absolute: bool = False,
                                  start_condition: str = '',
                                  pause_condition: str = '',
                                  end_condition: str = '',
                                  reset_condition: str = '') -> str:
        """
        True if tip_link is closer than threshold to goal_orientation
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=OrientationReached.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                goal_orientation=goal_orientation,
                                absolute=absolute,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                threshold=threshold)

    def add_pointing_at(self,
                        goal_point: PointStamped,
                        tip_link: Union[str, giskard_msgs.LinkName],
                        pointing_axis: Vector3Stamped,
                        root_link: Union[str, giskard_msgs.LinkName],
                        name: Optional[str] = None,
                        start_condition: str = '',
                        pause_condition: str = '',
                        end_condition: str = '',
                        reset_condition: str = '',
                        threshold: float = 0.01) -> str:
        """
        True if pointing_axis of tip_link is pointing at goal_point withing threshold.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=PointingAt.__name__,
                                name=name,
                                tip_link=tip_link,
                                goal_point=goal_point,
                                root_link=root_link,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                pointing_axis=pointing_axis,
                                threshold=threshold)

    def add_vectors_aligned(self,
                            root_link: Union[str, giskard_msgs.LinkName],
                            tip_link: Union[str, giskard_msgs.LinkName],
                            goal_normal: Vector3Stamped,
                            tip_normal: Vector3Stamped,
                            name: Optional[str] = None,
                            start_condition: str = '',
                            pause_condition: str = '',
                            end_condition: str = '',
                            reset_condition: str = '',
                            threshold: float = 0.01) -> str:
        """
        True if tip_normal of tip_link is aligned with goal_normal within threshold.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=VectorsAligned.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                goal_normal=goal_normal,
                                tip_normal=tip_normal,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                threshold=threshold)

    def add_end_motion(self,
                       start_condition: str,
                       name: Optional[str] = None) -> str:
        """
        Ends the motion execution/planning if all start_condition are True.
        Use this to describe when your motion should end.
        """
        return self.add_monitor(class_name=EndMotion.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition='',
                                end_condition='',
                                reset_condition='')

    def add_cancel_motion(self,
                          start_condition: str,
                          error: Optional[Exception] = None,
                          name: Optional[str] = None) -> str:
        """
        Cancels the motion if all start_condition are True and will make Giskard return the specified error code.
        Use this to describe when failure conditions.
        """
        if error is None:
            error = Exception(start_condition)
        error = msg_converter.exception_to_error_msg(error)
        return self.add_monitor(class_name=CancelMotion.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition='',
                                end_condition='',
                                reset_condition='',
                                exception=error)

    def add_check_trajectory_length(self, length: float = 30) -> str:
        """
        A monitor that cancels the motion if the trajectory is longer than max_trajectory_length.
        """
        self.max_trajectory_length_set = True
        length = self.add_monitor(name='max traj length',
                                  class_name=CheckMaxTrajectoryLength.__name__,
                                  length=length,
                                  start_condition='',
                                  pause_condition='',
                                  end_condition='',
                                  reset_condition='')
        return self.add_cancel_motion(start_condition=length, error=MaxTrajectoryLengthException('traj too long'),
                                      name='traj too long')

    def add_print(self,
                  message: str,
                  start_condition: str,
                  name: str) -> str:
        """
        Debugging Monitor.
        Print a message to the terminal if all start_condition are True.
        """
        return self.add_monitor(class_name=Print.__name__,
                                name=name,
                                message=message,
                                start_condition=start_condition,
                                pause_condition='',
                                end_condition=name,
                                reset_condition='')

    def add_sleep(self,
                  seconds: float,
                  name: Optional[str] = None,
                  start_condition: str = '') -> str:
        """
        Calls rospy.sleep(seconds) when start_condition are True and turns True itself afterward.
        """
        return self.add_monitor(class_name=Sleep.__name__,
                                name=name,
                                seconds=seconds,
                                start_condition=start_condition,
                                pause_condition='',
                                end_condition=name,
                                reset_condition='')

    def add_set_seed_configuration(self,
                                   seed_configuration: Dict[str, float],
                                   name: Optional[str] = None,
                                   group_name: Optional[str] = None,
                                   start_condition: str = '') -> str:
        """
        Only meant for use with projection. Changes the world state to seed_configuration before starting planning,
        without having to plan a motion to it like with add_joint_position
        """
        return self.add_monitor(class_name=SetSeedConfiguration.__name__,
                                seed_configuration=seed_configuration,
                                group_name=group_name,
                                name=name,
                                start_condition=start_condition,
                                pause_condition='',
                                end_condition=name,
                                reset_condition='')

    def add_set_seed_odometry(self,
                              base_pose: PoseStamped,
                              name: Optional[str] = None,
                              group_name: Optional[str] = None,
                              start_condition: str = '') -> str:
        """
        Only meant for use with projection. Overwrites the odometry transform with base_pose.
        """
        return self.add_monitor(class_name=SetOdometry.__name__,
                                group_name=group_name,
                                base_pose=base_pose,
                                name=name,
                                start_condition=start_condition,
                                pause_condition='',
                                end_condition=name,
                                reset_condition='')

    def add_alternator(self,
                       name: Optional[str] = None,
                       start_condition: str = '',
                       pause_condition: str = '',
                       end_condition: str = '',
                       reset_condition: str = '',
                       mod: int = 2) -> str:
        """
        Testing monitor.
        True if floor(trajectory_length) % mod == 0.
        """
        if name is None:
            name = Alternator.__name__ + f' % {mod}'
        return self.add_monitor(class_name=Alternator.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                mod=mod)

    def add_pulse(self,
                  after_ticks: int,
                  name: Optional[str] = None,
                  true_for_ticks: int = 1,
                  start_condition: str = '',
                  pause_condition: str = '',
                  end_condition: str = '',
                  reset_condition: str = '') -> str:
        """
        Testing monitor.
        Like add_alternator but as a PayloadMonitor.
        """
        return self.add_monitor(class_name=Pulse.__name__,
                                name=name,
                                true_for_ticks=true_for_ticks,
                                after_ticks=after_ticks,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_const_true(self,
                       name: Optional[str] = None,
                       start_condition: str = '',
                       pause_condition: str = '',
                       end_condition: str = '',
                       reset_condition: str = '', ) -> str:
        """
        Testing monitor.
        Like add_alternator but as a PayloadMonitor.
        """
        return self.add_monitor(class_name=TrueMonitor.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_const_false(self,
                        name: Optional[str] = None,
                        start_condition: str = '',
                        pause_condition: str = '',
                        end_condition: str = '',
                        reset_condition: str = '') -> str:
        """
        Testing monitor.
        Like add_alternator but as a PayloadMonitor.
        """
        return self.add_monitor(class_name=FalseMonitor.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_vectors_perpendicular(self,
                                  root_link: Union[str, giskard_msgs.LinkName],
                                  tip_link: Union[str, giskard_msgs.LinkName],
                                  reference_normal: Vector3Stamped,
                                  tip_normal: Vector3Stamped,
                                  name: Optional[str] = None,
                                  start_condition: str = '',
                                  pause_condition: str = '',
                                  end_condition: str = '',
                                  reset_condition: str = '',
                                  threshold: float = 0.01) -> str:
        """
        True if tip_normal of tip_link is perpendicular to goal_normal within threshold.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=PerpendicularMonitor.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                reference_normal=reference_normal,
                                tip_normal=tip_normal,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                threshold=threshold)

    def add_angle(self,
                  root_link: Union[str, giskard_msgs.LinkName],
                  tip_link: Union[str, giskard_msgs.LinkName],
                  reference_vector: Vector3Stamped,
                  tip_vector: Vector3Stamped,
                  lower_angle: float,
                  upper_angle: float,
                  name: Optional[str] = None,
                  start_condition: str = '',
                  pause_condition: str = '',
                  end_condition: str = '',
                  reset_condition: str = '') -> str:
        """
        True if angle between tip_vector and reference_vector is within lower and upper angle.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=AngleMonitor.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                reference_vector=reference_vector,
                                tip_vector=tip_vector,
                                lower_angle=lower_angle,
                                upper_angle=upper_angle,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_height(self,
                   root_link: Union[str, giskard_msgs.LinkName],
                   tip_link: Union[str, giskard_msgs.LinkName],
                   reference_point: PointStamped,
                   tip_point: PointStamped,
                   lower_limit: float,
                   upper_limit: float,
                   name: Optional[str] = None,
                   start_condition: str = '',
                   pause_condition: str = '',
                   end_condition: str = '',
                   reset_condition: str = '') -> str:
        """
        True if distance along the z-axis of root_link between tip_point and reference_point
        is within lower and upper limit.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=HeightMonitor.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                reference_point=reference_point,
                                tip_point=tip_point,
                                lower_limit=lower_limit,
                                upper_limit=upper_limit,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    def add_distance(self,
                     root_link: Union[str, giskard_msgs.LinkName],
                     tip_link: Union[str, giskard_msgs.LinkName],
                     reference_point: PointStamped,
                     tip_point: PointStamped,
                     lower_limit: float,
                     upper_limit: float,
                     name: Optional[str] = None,
                     start_condition: str = '',
                     pause_condition: str = '',
                     end_condition: str = '',
                     reset_condition: str = '') -> str:
        """
        True if distance between tip_point and reference_point on the plane (that has the z-axis of
        root_link as a normal vector) is within lower and upper limit.
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        return self.add_monitor(class_name=DistanceMonitor.__name__,
                                name=name,
                                root_link=root_link,
                                tip_link=tip_link,
                                reference_point=reference_point,
                                tip_point=tip_point,
                                lower_limit=lower_limit,
                                upper_limit=upper_limit,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition)

    # Start of SuTuRo-specific monitors

    def add_force_torque(self,
                         threshold_enum: int,
                         object_type: Optional[str] = None,
                         topic: str = '/filtered_raw/diff',
                         name: Optional[str] = None,
                         stay_true: bool = True,
                         start_condition: str = '',
                         pause_condition: str = '',
                         end_condition: str = '',
                         reset_condition: str = ''):
        """
        Can be used if planning wants to use their own grasping and placing actions, only adds force_torque_monitor
        without manipulations grasping/placing goals

        :param threshold_enum: ID of the threshold to be used for the Force-Torque Monitor, options can be found in giskardpy/data_types/suturo_types.py
        :param object_type: Name of the object that is being placed, options can be found in giskardpy/data_types/suturo_types.py
        :param topic: name of the topic that the monitor should subscribe to, is hardcoded as '/filtered_raw/diff'
        :param name: name of the monitor, is optional, so can be left empty
        :param stay_true: whether the monitor should stay active until it is finished or not
        :param start_condition: condition for when the monitor should start checking for thresholds
        """
        if ((threshold_enum == ForceTorqueThresholds.GRASP.value or threshold_enum == ForceTorqueThresholds.PLACE.value)
                and object_type is None):
            raise MonitorInitalizationException('object_type not optional for GRASP and PLACE')

        return self.add_monitor(class_name=PayloadForceTorque.__name__,
                                name=name,
                                start_condition=start_condition,
                                pause_condition=pause_condition,
                                end_condition=end_condition,
                                reset_condition=reset_condition,
                                stay_true=stay_true,
                                topic=topic,
                                threshold_enum=threshold_enum,
                                object_type=object_type)

    def add_payload_lidar(self,
                          start_condition: str = '',
                          topic: str = 'hsrb/base_scan',
                          name: Optional[str] = None,
                          frame_id: Optional[str] = 'base_range_sensor_link',
                          laser_distance_threshold_width: Optional[float] = 0.8,
                          laser_distance_threshold: Optional[float] = 0.5):
        """
        A monitor that detects points within a threshold of the robot via laser_scanner.
        Is True, when a point is detected within the threshold, False otherwise
        """
        if name is None:
            name = LidarPayloadMonitor.__name__ + f'_{topic}'
        return self.add_monitor(class_name=LidarPayloadMonitor.__name__,
                                name=name,
                                start_condition=start_condition,
                                topic=topic,
                                frame_id=frame_id,
                                laser_distance_threshold_width=laser_distance_threshold_width,
                                laser_distance_threshold=laser_distance_threshold)

    def add_open_hsr_gripper(self,
                             start_condition: str = '',
                             name: Optional[str] = None) -> str:
        """
        The monitor will send a force to the HSR's gripper to open it.
        """
        from giskardpy.motion_statechart.monitors.hsr_gripper import OpenHsrGripper
        name = name or OpenHsrGripper.__name__
        return self.add_monitor(class_name=OpenHsrGripper.__name__,
                                name=name,
                                start_condition=start_condition,
                                end_condition=name)

    def add_close_hsr_gripper(self,
                              start_condition: str = '',
                              name: Optional[str] = None) -> str:
        """
        The monitor will send a force to the HSR's gripper to close it.
        """
        from giskardpy.motion_statechart.monitors.hsr_gripper import CloseHsrGripper
        name = name or CloseHsrGripper.__name__
        return self.add_monitor(class_name=CloseHsrGripper.__name__,
                                name=name,
                                start_condition=start_condition,
                                end_condition=name)

    def add_handle_offset(self,
                          root_link: Union[str, giskard_msgs.LinkName] = giskard_msgs.LinkName('map', ''),
                          tip_link: Union[str, giskard_msgs.LinkName] = giskard_msgs.LinkName('hand_camera_frame', ''),
                          threshold: float = 50,
                          weight: Optional[float] = None,
                          name: Optional[str] = None,
                          start_condition: str = '',
                          pause_condition: str = '',
                          end_condition: str = ''):
        """
        Adds tip_link correction to handle using visual servoing with robokudo handle_offset pipeline

        :param root_link: root link of the kinematic chain
        :param tip_link: tip link of the kinematic chain
        :param threshold: threshold in pixel that determines when the handle is correctly orientated
        :param weight:
        :param name:
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        if isinstance(root_link, str):
            root_link = giskard_msgs.LinkName(name=root_link)
        if isinstance(tip_link, str):
            tip_link = giskard_msgs.LinkName(name=tip_link)
        self.add_monitor(class_name=HandleOffsetCorrection.__name__,
                         root_link=root_link,
                         tip_link=tip_link,
                         threshold=threshold,
                         name=name,
                         weight=weight,
                         start_condition=start_condition,
                         pause_condition=pause_condition,
                         end_condition=end_condition)

    def add_handle_offset_reset(self,
                                reset_joint: Union[str, giskard_msgs.LinkName],
                                parent_T_child: PoseStamped,
                                child_frame: Union[str, giskard_msgs.LinkName],
                                name: Optional[str] = None,
                                start_condition: str = '',
                                pause_condition: str = '',
                                end_condition: str = ''):
        """
        Resets given joints transform to parent_T_child, used to reset joint after handle_offset_correction

        :param reset_joint: Joint to reset
        :param parent_T_child: parent_T_child TransMatrix to reset joint to
        :param child_frame: child_frame used in parent_T_child
        :param name:
        :param start_condition: expression that starts goal
        :param pause_condition: expression that pauses goal
        :param end_condition: expression that ends goal
        """
        if isinstance(reset_joint, str):
            reset_joint = giskard_msgs.LinkName(name=reset_joint)
        if isinstance(child_frame, str):
            child_frame = giskard_msgs.LinkName(name=child_frame)
        self.add_monitor(class_name=OffsetCorrectionReset.__name__,
                         reset_joint=reset_joint,
                         parent_T_child=parent_T_child,
                         child_frame=child_frame,
                         name=name,
                         start_condition=start_condition,
                         pause_condition=pause_condition,
                         end_condition=end_condition)


class GiskardWrapper:
    last_execution_state: ExecutionState
    last_feedback: MoveFeedback = None
    last_execution_state: ExecutionState = None

    def __init__(self, node_name: str = 'giskard', check_controller: bool = True):
        """
        Python wrapper for the ROS interface of Giskard.
        :param node_name: node name of Giskard
        """
        self.list_controller_srv = None
        self.world = WorldWrapper(node_name)
        self.monitors = MonitorWrapper(self)
        self.motion_goals = MotionGoalWrapper(self)
        self.clear_motion_goals_and_monitors()
        giskard_topic = f'{node_name}/command'
        self._client = SimpleActionClient(giskard_topic, MoveAction)
        self._client.wait_for_server()
        self.clear_motion_goals_and_monitors()
        rospy.sleep(.3)

        # if check_controller and self.world.get_control_mode() == ControlModes.close_loop:
        #    self.setup_controllers()

    def setup_controllers(self):
        start_con = ['realtime_body_controller_real']
        stop_con = ['arm_trajectory_controller', 'head_trajectory_controller']
        switch_controllers_srv_name = '/hsrb/controller_manager/switch_controller'
        list_controllers_srv_name = '/hsrb/controller_manager/list_controllers'
        self.list_controller_srv = rospy.ServiceProxy(name=list_controllers_srv_name,
                                                      service_class=ListControllers)
        self.set_closed_loop_controllers(stop=stop_con,
                                         start=start_con,
                                         switch_controller_srv=switch_controllers_srv_name)
        if not self.check_controllers_active(
                stopped_controllers=stop_con,
                running_controllers=start_con):
            raise Exception(f'Controllers are configured incorrectly. Look at rqt_controller_manager.')

    def set_closed_loop_controllers(self,
                                    stop: Optional[List] = None,
                                    start: Optional[List] = None,
                                    switch_controller_srv: Optional[
                                        str] = '/hsrb/controller_manager/switch_controller') -> SwitchControllerResponse:
        """
        Start and Stop controllers for via the designated switch_controller service
        """

        if stop is None:
            stop = ['arm_trajectory_controller', 'head_trajectory_controller']
        if start is None:
            start = ['realtime_body_controller_real']

        start_controllers = start
        stop_controllers = stop
        strictness: int = 1
        start_asap: bool = False
        timeout: float = 0.0

        rospy.wait_for_service(switch_controller_srv)
        srv_switch_con = rospy.ServiceProxy(name=switch_controller_srv,
                                            service_class=SwitchController)

        resp: SwitchControllerResponse = srv_switch_con(start_controllers,
                                                        stop_controllers,
                                                        strictness,
                                                        start_asap,
                                                        timeout)
        return resp

    def check_controllers_active(self,
                                 stopped_controllers: Optional[List] = None,
                                 running_controllers: Optional[List] = None):
        """
        Checks if the arm_trajectory_controller and head_trajectory_controller are stopped
        and the realtime_body_controller_real is running

        :param stopped_controllers: controllers that have to be in state stopped or initialized
        :param running_controllers: controllers that have to be in state running
        """
        if stopped_controllers is None:
            stopped_controllers = []
        if running_controllers is None:
            running_controllers = []

        resp: ListControllersResponse = self.list_controller_srv()
        controller_dict = {controller.name: controller for controller in resp.controller}

        if (all(controller_dict[con].state == 'stopped' or controller_dict[con].state == 'initialized' for con in
                stopped_controllers) and all(controller_dict[con].state == 'running' for con in running_controllers)):
            return True
        return False

    def set_avoid_name_conflict(self, value: bool):
        self.avoid_name_conflict = value
        self.monitors.avoid_name_conflict = value
        self.motion_goals.avoid_name_conflict = value

    def set_conditions(self, node_name: str,
                       start_condition: str,
                       pause_condition: str,
                       end_condition: str,
                       reset_condition: str):
        self.update_start_condition(node_name, start_condition)
        self.update_pause_condition(node_name, pause_condition)
        self.update_end_condition(node_name, end_condition)
        self.update_reset_condition(node_name, reset_condition)

    def update_start_condition(self, node_name: str, condition: str) -> None:
        self.motion_statechart_nodes[node_name].start_condition = condition

    def update_reset_condition(self, node_name: str, condition: str) -> None:
        self.motion_statechart_nodes[node_name].reset_condition = condition

    def update_pause_condition(self, node_name: str, condition: str) -> None:
        self.motion_statechart_nodes[node_name].pause_condition = condition

    def update_end_condition(self, node_name: str, condition: str) -> None:
        self.motion_statechart_nodes[node_name].end_condition = condition

    def quote_node_names(self, condition: str) -> str:
        operators = {'and', 'or', 'not', '(', ')'}
        pattern = r'(\b(?:and|or|not)\b|\(|\))'
        tokens = re.split(pattern, condition)
        result = []
        for token in tokens:
            if token in operators:
                result.append(token)
            elif token.strip() == '':
                result.append(token)
            else:
                # Check if token is already quoted
                stripped = token.strip()
                if (stripped.startswith('"') and stripped.endswith('"')) or \
                        (stripped.startswith("'") and stripped.endswith("'")):
                    result.append(token)
                else:
                    # Wrap stripped token in quotes, and preserve leading/trailing spaces
                    leading_spaces = len(token) - len(token.lstrip())
                    trailing_spaces = len(token) - len(token.rstrip())
                    leading = token[:leading_spaces]
                    trailing = token[len(token.rstrip()):]
                    result.append(f'{leading}"{stripped}"{trailing}')
        return ''.join(result)

    @property
    def motion_statechart_nodes(self) -> Dict[str, MotionStatechartNode]:
        return {**self.motion_goals.motion_graph_nodes,
                **self.monitors.motion_graph_nodes}

    @property
    def num_nodes(self) -> int:
        return len(self.motion_statechart_nodes)

    def add_default_end_motion_conditions(self) -> None:
        """
        1. Adds a local minimum reached monitor and adds it as an end_condition to all previously defined motion goals.
        2. Adds an end motion monitor, start_condition = all previously defined monitors are True.
        3. Adds a cancel motion monitor, start_condition = local minimum reached mit not all other monitors are True.
        4. Adds a max trajectory length monitor, if one wasn't added already.
        """
        local_min_reached_monitor_name = self.monitors.add_local_minimum_reached('local min')
        # for node in self.motion_statechart_nodes.values():
        #     if node.end_condition and node.name != local_min_reached_monitor_name:
        #         node.end_condition = f'({node.end_condition}) and {local_min_reached_monitor_name}'
        #     else:
        #         node.end_condition = local_min_reached_monitor_name
        end_motion_condition = local_min_reached_monitor_name
        monitor_part = self.monitors.get_anded_nodes(add_nodes_without_end_condition=False)
        if len(monitor_part) > 0:
            end_motion_condition += f' and {monitor_part}'
        motion_goal_part = self.motion_goals.get_anded_nodes(add_nodes_without_end_condition=False)
        if len(motion_goal_part) > 0:
            if len(end_motion_condition) > 0:
                end_motion_condition += f' and {motion_goal_part}'
            else:
                end_motion_condition = motion_goal_part
        self.monitors.add_end_motion(start_condition=end_motion_condition)
        # self.monitors.add_cancel_motion(start_condition=local_min_reached_monitor_name,
        #                                 error=LocalMinimumException(f'local minimum reached'))
        if not self.monitors.max_trajectory_length_set:
            self.monitors.add_check_trajectory_length()
        self.monitors.max_trajectory_length_set = False

    def add_end_on_local_minimum(self) -> None:
        local_min_reached_monitor_name = self.monitors.add_local_minimum_reached('local min reached')
        self.monitors.add_end_motion(start_condition=local_min_reached_monitor_name)

    def add_lidar_hold_condition(self) -> None:
        """
        1. Adds a lidar payload monitor and adds it as pause_condition to all previously defined motions goals
        """
        lidar_monitor_name = self.monitors.add_payload_lidar(laser_distance_threshold_width=0.3,
                                                             laser_distance_threshold=0.35)
        for goal in self.motion_goals._goals:
            if goal.pause_condition:
                goal.pause_condition = f'{goal.pause_condition} and {lidar_monitor_name}'
            else:
                goal.pause_condition = lidar_monitor_name

    @property
    def robot_name(self):
        return self.world.robot_name

    def clear_motion_goals_and_monitors(self):
        """
        Removes all move commands from the current goal, collision entries are left untouched.
        """
        self.motion_goals.reset()
        self.monitors.reset()

    def execute(self, wait: bool = True) -> MoveResult:
        """
        :param wait: this function blocks if wait=True
        :return: result from giskard
        """
        result = self._send_action_goal(MoveGoal.EXECUTE, wait)
        if result:
            exception = msg_converter.error_msg_to_exception(result.error)
            if exception is not None:
                raise exception
            return result

    def projection(self, wait: bool = True) -> MoveResult:
        """
        Plans, but doesn't execute the goal. Useful, if you just want to look at the planning ghost.
        :param wait: this function blocks if wait=True
        :return: result from Giskard
        """
        return self._send_action_goal(MoveGoal.PROJECTION, wait)

    def _send_action_goal(self, goal_type: int, wait: bool = True) -> Optional[MoveResult]:
        """
        Send goal to Giskard. Use this if you want to specify the goal_type, otherwise stick to wrappers like
        plan_and_execute.
        :param goal_type: one of the constants in MoveGoal
        :param wait: blocks if wait=True
        :return: result from Giskard
        """
        goal = self._create_action_goal()
        goal.type = goal_type
        if wait:
            self._client.send_goal_and_wait(goal)
            result = self._client.get_result()
            self.last_execution_state = result
            return result
        else:
            self._client.send_goal(goal, feedback_cb=self._feedback_cb)

    def _create_action_goal(self) -> MoveGoal:
        if not self.motion_goals._collision_entries:
            self.motion_goals.avoid_all_collisions()
        self.motion_goals._add_collision_entries_as_goals()
        action_goal = MoveGoal()
        templated_and_tasks = self._quote_conditions(self.monitors.motion_graph_nodes)
        monitors = self._quote_conditions(self.motion_goals.motion_graph_nodes)
        action_goal.nodes = templated_and_tasks + monitors
        self.clear_motion_goals_and_monitors()
        return action_goal

    def _quote_conditions(self, nodes: Dict[str, MotionStatechartNode]) -> List[MotionStatechartNode]:
        result = []
        for node in nodes.values():
            node.start_condition = self.quote_node_names(node.start_condition)
            node.pause_condition = self.quote_node_names(node.pause_condition)
            node.end_condition = self.quote_node_names(node.end_condition)
            node.reset_condition = self.quote_node_names(node.reset_condition)
            result.append(node)
        return result

    def interrupt(self):
        """
        Stops the goal that was last sent to Giskard.
        """
        self._client.cancel_goal()

    def cancel_all_goals(self):
        """
        Stops any goal that Giskard is processing and attempts to halt the robot, even those not send from this client.
        """
        self._client.cancel_all_goals()

    def get_result(self, timeout: rospy.Duration = rospy.Duration()) -> MoveResult:
        """
        Waits for Giskard result and returns it. Only used when plan_and_execute was called with wait=False
        :param timeout: how long to wait
        """
        if not self._client.wait_for_result(timeout):
            raise TimeoutError('Timeout while waiting for goal.')
        return self._client.get_result()

    def _feedback_cb(self, msg: MoveFeedback):
        self.last_feedback = msg

    def get_end_motion_reason(self, move_result: Optional[MoveResult] = None, show_all: bool = False) \
            -> Dict[str, bool]:
        """
        Analyzes a MoveResult msg to return a list of all monitors that hindered the EndMotion Monitors from becoming active.
        Uses the last received MoveResult msg from execute() or projection() when not explicitly given.
        :param move_result: the move_result msg to analyze
        :param show_all: returns the state of all monitors when show_all==True
        :return: Dict with monitor name as key and True or False as value
        """
        if not move_result and not self.last_execution_state:
            raise Exception('No MoveResult available to analyze')
        elif not move_result:
            execution_state = self.last_execution_state
        else:
            execution_state = move_result.execution_state

        result = {}
        if show_all:
            return {monitor.name: state for monitor, state in
                    zip(execution_state.monitors, execution_state.monitor_state)}

        failedEndMotion_ids = []
        monitor: MotionStatechartNode
        for idx, monitor in enumerate(execution_state.monitors):
            if monitor.class_name == 'EndMotion' and execution_state.monitor_state[idx] != 1:
                failedEndMotion_ids.append(idx)

        if len(failedEndMotion_ids) == 0:
            # the end motion was successful
            return result

        def search_for_monitor_values_in_start_condition(start_condition: str):
            res = []
            for monitor, state in zip(execution_state.monitors, execution_state.monitor_state):
                if f'\'{monitor.name}\'' in start_condition or f'"{monitor.name}"' in start_condition and state != 1:
                    res.append(monitor)
            return res

        for endMotion_idx in failedEndMotion_ids:
            start_condition = execution_state.monitors[endMotion_idx].start_condition
            false_monitors = search_for_monitor_values_in_start_condition(start_condition=start_condition)
            # repeatedly search for all inactive monitors in all start_conditions directly
            # connected to the endMotion start_condition
            for idx, false_monitor in enumerate(false_monitors):
                if false_monitors[idx].start_condition != '1.0':
                    false_monitors.extend(
                        search_for_monitor_values_in_start_condition(false_monitor.start_condition))

            for mon in false_monitors:
                result[mon.name] = False

        return result

    # SuTuRo Goals start here! (Only add SuTuRo goals which actually need monitors for stopping conditions etc.)
    def monitor_placing(self,
                        align: str,
                        grasp: str,
                        threshold_enum: int,
                        goal_pose: PoseStamped,
                        object_type: str = "",
                        tip_link: str = 'hand_palm_link',
                        velocity: float = 0.02):
        """
        adds monitor functionality for the Placing motion goal, goal now stops if force_threshold is overstepped,
        which means the HSR essentially stops automatically after placing the object.
        (not currently in use, since planning doesn't use our pickup/placing goals)

        :param align: alignment of action, should currently be either "vertical" or an empty string if not needed
        :param grasp: the direction from which the HSR should Grasp an object, in case of this method it should be direction the HSR is placing from
        :param goal_pose: where the object should be placed
        :param threshold_enum: Name of the threshold to be used for the Force-Torque Monitor, options can be found in giskardpy/data_types/suturo_types.py
        :param object_type: Name of the object that is being placed, options can be found in giskardpy/data_types/suturo_types.py
        :param tip_link: name of the tip link, pre-defined as "hand_palm_link"
        :param velocity: the velocity that this action should be executed with
        """

        sleep = self.monitors.add_sleep(1.5)
        force_torque_trigger = self.monitors.add_monitor(class_name=PayloadForceTorque.__name__,
                                                         name=PayloadForceTorque.__name__,
                                                         start_condition='',
                                                         threshold_name=threshold_enum,
                                                         object_type=object_type)

        self.motion_goals.add_motion_goal(class_name=Placing.__name__,
                                          goal_pose=goal_pose,
                                          align=align,
                                          grasp=grasp,
                                          tip_link=tip_link,
                                          velocity=velocity,
                                          end_condition=f'{force_torque_trigger} and {sleep}')

        local_min = self.monitors.add_local_minimum_reached()

        self.monitors.add_cancel_motion(local_min, ObjectForceTorqueThresholdException('force violated'))
        self.monitors.add_end_motion(start_condition=force_torque_trigger)
        self.monitors.add_max_trajectory_length(100)

    def monitor_grasp_carefully(self,
                                goal_pose: PoseStamped,
                                align: str,
                                grasp: str,
                                threshold_enum: int,
                                reference_frame_alignment: Optional[str] = None,
                                object_name: str = "",
                                object_type: str = "",
                                root_link: Optional[str] = None,
                                tip_link: Optional[str] = None):
        """
        adds monitor functionality to the reaching goal, thus making the original GraspCarefully motion goal redundant.
        The goal now stops if force_threshold/torque_threshold is undershot,
        which means it essentially stops automatically if the HSR for example slips off of a door handle while trying
        to open doors or fails to properly grip an object.
        (not currently in use, since planning doesn't use our pickup/placing goals)

        :param goal_pose: where the object should be placed
        :param align: alignment of action, should currently be either "vertical" or an empty string if not needed
        :param grasp: the direction from which the HSR should Grasp an object
        :param reference_frame_alignment: reference frame to be used for alignment, can be left empty
        :param object_name: name of the object, needed for offset calculation of reaching goal
        :param object_type: Name of the object that is being placed, options can be found in suturo_types.py
        :param threshold_name: Name of the threshold to be used for the Force-Torque Monitor, options can be found in suturo_types.py
        :param root_link: root_link to be used, is optional, so should normally be left empty
        :param tip_link: name of the tip link, is optional, so can be left empty
        """
        sleep = self.monitors.add_sleep(1.5)
        # gripper_open = self.monitors.add_open_hsr_gripper()
        force_torque_trigger = self.monitors.add_monitor(class_name=PayloadForceTorque.__name__,
                                                         name=PayloadForceTorque.__name__,
                                                         start_condition='',
                                                         threshold_name=threshold_enum,
                                                         object_type=object_type)

        self.motion_goals.add_motion_goal(class_name=Reaching.__name__,
                                          goal_pose=goal_pose,
                                          grasp=grasp,
                                          align=align,
                                          reference_frame_alignment=reference_frame_alignment,
                                          object_name=object_name,
                                          root_link=root_link,
                                          tip_link=tip_link,
                                          end_condition=f'{force_torque_trigger} and {sleep}')

        local_min = self.monitors.add_local_minimum_reached()

        self.monitors.add_cancel_motion(local_min, ObjectForceTorqueThresholdException('force violated'))
        self.monitors.add_end_motion(start_condition=force_torque_trigger)
        self.monitors.add_max_trajectory_length(100)

    def monitor_force_torque_check(self,
                                   goal_pose: PoseStamped,
                                   tip_link: str,
                                   root_link: str,
                                   threshold_enum: int,
                                   position_threshold: float,
                                   orientation_threshold: float,
                                   object_type: str = ""):
        """
        force_torque_monitor used for grasping, activates when the hsr closes it's gripper and then checks
        via force_torque whether the necessary threshold has been overshot, thus essentially checking if
        object has successfully been grasped/ placed.
        This function also includes cartesian monitors from planning so that we can properly bind the
        force_torque_monitor to the relevant motions, so that they get canceled if threshold is undershot.

        :param object_type: type of the object that is being transported, needed to determine correct threshold
        :param threshold_name: name of the motion, should be transport in this case, but the corresponding enum doesn't exist yet
        :param goal_pose: goal pose for cartesian monitor
        :param tip_link: tip link for cartesian monitor
        :param root_link: root link for cartesian monitor
        :param position_threshold: position threshold for cartesian monitor
        :param orientation_threshold: orientation threshold for cartesian monitor
        """

        cart_monitor1 = self.monitors.add_cartesian_pose(root_link=root_link, tip_link=tip_link,
                                                         goal_pose=goal_pose,
                                                         position_threshold=position_threshold,
                                                         orientation_threshold=orientation_threshold,
                                                         name='cart goal 1')
        end_monitor = self.monitors.add_local_minimum_reached(start_condition=cart_monitor1)

        self.motion_goals.add_cartesian_pose(name='g1', root_link=root_link, tip_link=tip_link,
                                             goal_pose=goal_pose,
                                             end_condition=cart_monitor1)

        self.motion_goals.avoid_all_collisions()
        self.motion_goals.allow_collision(group1='gripper', group2=CollisionEntry.ALL)
        # gripper_closed = self.monitors.add_close_hsr_gripper()

        mon = self.monitors.add_monitor(class_name=PayloadForceTorque.__name__,
                                        name=PayloadForceTorque.__name__,
                                        topic='/filtered_raw/diff',
                                        start_condition='',
                                        threshold_enum=threshold_enum,
                                        object_type=object_type)

        sleep = self.monitors.add_sleep(1)
        # local_min = self.monitors.add_local_minimum_reached(name='force_torque_local_min')

        self.monitors.add_cancel_motion(f'not {mon} and {sleep} ',
                                        ObjectForceTorqueThresholdException('force violated'))
        self.monitors.add_end_motion(start_condition=f'{mon} and {sleep} and {end_monitor}')
        self.execute()
        # self.monitors.add_max_trajectory_length(100)

    # TODO: put logic into giskard Interface of Planning where Monitors and Motions are used
    # also other hsrb specific methods
    def grasp_bar_offset_goal(self,
                              bar_center: PointStamped,
                              bar_axis: Vector3Stamped,
                              bar_length: float,
                              tip_link: str,
                              tip_grasp_axis: Vector3Stamped,
                              root_link: str,
                              grasp_axis_offset: Vector3Stamped,
                              tip_group: Optional[str] = None,
                              root_group: Optional[str] = None,
                              reference_linear_velocity: Optional[float] = None,
                              reference_angular_velocity: Optional[float] = None,
                              weight: float = WEIGHT_ABOVE_CA,
                              add_monitor: bool = True,
                              **kwargs: goal_parameter):
        """
        Like a CartesianPose but with more freedom.
        tip_link is allowed to be at any point along bar_axis, that is without bar_center +/- bar_length.
        It will align tip_grasp_axis with bar_axis, but allows rotation around it.
        :param root_link: root link of the kinematic chain
        :param tip_link: tip link of the kinematic chain
        :param tip_grasp_axis: axis of tip_link that will be aligned with bar_axis
        :param bar_center: center of the bar to be grasped
        :param bar_axis: alignment of the bar to be grasped
        :param bar_length: length of the bar to be grasped
        :param grasp_axis_offset: offset of the tip_link to the bar_center
        :param root_group: if root_link is not unique, search in this group for matches
        :param tip_group: if tip_link is not unique, search in this group for matches
        :param reference_linear_velocity: m/s
        :param reference_angular_velocity: rad/s
        :param weight:
        :param add_monitor: if True, adds a monitor as end_condition to check if the goal was reached.
        """
        end_condition = ''
        if add_monitor:
            monitor_name1 = self.monitors.add_distance_to_line(root_link=root_link,
                                                               tip_link=tip_link,
                                                               center_point=bar_center,
                                                               line_axis=bar_axis,
                                                               line_length=bar_length)
            monitor_name2 = self.monitors.add_vectors_aligned(root_link=root_link,
                                                              tip_link=tip_link,
                                                              goal_normal=bar_axis,
                                                              tip_normal=tip_grasp_axis)
            end_condition = f'{monitor_name1} and {monitor_name2}'
        self.motion_goals.add_grasp_bar_offset(end_condition=end_condition,
                                               root_link=root_link,
                                               tip_link=tip_link,
                                               tip_grasp_axis=tip_grasp_axis,
                                               bar_center=bar_center,
                                               bar_axis=bar_axis,
                                               bar_length=bar_length,
                                               grasp_axis_offset=grasp_axis_offset,
                                               root_group=root_group,
                                               tip_group=tip_group,
                                               reference_linear_velocity=reference_linear_velocity,
                                               reference_angular_velocity=reference_angular_velocity,
                                               weight=weight,
                                               **kwargs)

    def hsrb_dishwasher_door_handle_grasp(self,
                                          handle_frame_id: str,
                                          grasp_bar_offset: float = 0.0,
                                          root_link: str = 'map',
                                          tip_link: str = 'hand_gripper_tool_frame'):
        """
        :param handle_frame_id: frame id of the dishwashers handle
        :param grasp_bar_offset: offset that is applied to the grasping axis
        :param root_link: root link, in this case the map link
        :param tip_link: tip link of the gripper, in this case the 'hand_gripper_tool_frame'
        """

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip_link
        tip_grasp_axis.vector.x = 1

        grasp_axis_offset = Vector3Stamped()
        grasp_axis_offset.header.frame_id = handle_frame_id
        grasp_axis_offset.vector.x = -grasp_bar_offset

        self.grasp_bar_offset_goal(root_link=root_link,
                                   tip_link=tip_link,
                                   tip_grasp_axis=tip_grasp_axis,
                                   bar_center=bar_center,
                                   bar_axis=bar_axis,
                                   bar_length=.4,
                                   grasp_axis_offset=grasp_axis_offset)

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = tip_link
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1

        self.motion_goals.add_align_planes(tip_link=tip_link,
                                           tip_normal=x_gripper,
                                           goal_normal=x_goal,
                                           root_link=root_link)

    def pre_pose_shelf_open(self,
                            offset_x: float = 0.03,
                            offset_y: float = 0.01,
                            offset_z: float = -0.15,
                            left_handle: str = 'shelf_billy:shelf_billy:shelf_door_left:handle',
                            left_door: str = 'shelf_billy:shelf_billy:shelf_door_left',
                            group_name: str = 'iai_kitchen',
                            orientation: np.array = None):
        """
        Pre-Pose for opening the shelf

        :param offset_x: Depth offset for grasping pose of the handle
        :param offset_y: Width offset for grasping pose of the handle
        :param offset_z: Height offset for grasping pose of the handle
        :param left_handle: Left door handle of the shelf
        :param left_door: Main Link of the left door
        """
        if orientation is None:
            orientation = np.array([[-1, 0, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 0, 1]])
        if left_door not in self.world.get_group_names():
            self.world.register_group(new_group_name=left_door,
                                      root_link_name=giskard_msgs.LinkName(name=left_door,
                                                                           group_name=group_name))

        first_goal = PoseStamped()
        first_goal.header.frame_id = left_handle
        first_goal.pose.position.x = offset_x
        first_goal.pose.position.y = offset_y
        first_goal.pose.position.z = offset_z
        first_goal.pose.orientation = Quaternion(*quaternion_from_matrix(orientation))

        pre_grasp_reached = self.monitors.add_cartesian_pose(name='pre first goal monitor',
                                                             goal_pose=first_goal,
                                                             tip_link='hand_gripper_tool_frame',
                                                             root_link='map',
                                                             position_threshold=0.08,
                                                             orientation_threshold=0.04)
        self.motion_goals.avoid_all_collisions(end_condition=pre_grasp_reached)
        self.motion_goals.allow_collision(group1='gripper', group2=left_door, start_condition=pre_grasp_reached)

        grasp_reached = self.monitors.add_cartesian_pose(name='first goal monitor',
                                                         goal_pose=first_goal,
                                                         tip_link='hand_gripper_tool_frame',
                                                         root_link='map',
                                                         position_threshold=0.01)

        self.motion_goals.add_cartesian_pose(goal_pose=first_goal,
                                             tip_link='hand_gripper_tool_frame',
                                             root_link='map',
                                             reference_linear_velocity=0.05,
                                             reference_angular_velocity=0.25,
                                             weight=WEIGHT_BELOW_CA,
                                             end_condition=grasp_reached)

        self.monitors.add_end_motion(start_condition=grasp_reached)

    def open_shelf_door(self,
                        left_handle: str = 'shelf_billy:shelf_billy:shelf_door_left:handle',
                        left_door: str = 'shelf_billy:shelf_billy:shelf_door_left',
                        left_door_hinge: str = 'shelf_billy:shelf_billy:shelf_door_left:joint'):
        """
        Opens the shelf door.
        Requires the pre-pose to be reached and the gripper to be closed

        :param left_handle: Left door handle of the shelf
        :param left_door: Main Link of the left door
        :param left_door_hinge: Hinge Joint of the left door
        """
        if left_door not in self.world.get_group_names():
            self.world.register_group(new_group_name=left_door,
                                      root_link_name=giskard_msgs.LinkName(name=left_door,
                                                                           group_name='iai_kitchen'))

        self.motion_goals.allow_collision(group1='gripper', group2=left_door)

        self.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame',
                                             environment_link=left_handle,
                                             goal_joint_state=-1.5)

        open_door = self.monitors.add_joint_position({left_door_hinge: -1.5})
        self.monitors.add_end_motion(open_door)

    def hsrb_dishwasher_test(self, handle_frame_id: str, hinge_joint: str, door_hinge_frame_id: str):
        """
        Opening motion of dishwasher
        Specific implementation used for robocup with grasping handle
        """
        # TODO: make parameters better available
        root_link = 'map'
        tip_link = 'hand_gripper_tool_frame'
        grasp_bar_offset = 0.2
        goal_angle_half = 0.5
        goal_angle_full = 1.35
        bar_length = 0.1
        after_force_retract = 0.015
        env_name = 'iai_kitchen'
        allow_collision_group = 'arm'

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.y = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis_bar = Vector3Stamped()
        tip_grasp_axis_bar.header.frame_id = tip_link
        tip_grasp_axis_bar.vector.x = 1

        grasp_axis_offset = Vector3Stamped()
        grasp_axis_offset.header.frame_id = handle_frame_id
        grasp_axis_offset.vector.x = -grasp_bar_offset

        grasp_axis_offset_pre = Vector3Stamped()
        grasp_axis_offset_pre.header.frame_id = handle_frame_id
        grasp_axis_offset_pre.vector.x = 0.1

        tip_grasp_axis_push = Vector3Stamped()
        tip_grasp_axis_push.header.frame_id = tip_link
        tip_grasp_axis_push.vector.y = 1

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = tip_link
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1

        first_open = self.monitors.add_open_hsr_gripper(name='first open gripper')

        local_min_pre_grasp = self.monitors.add_local_minimum_reached(name='local min pre grasp',
                                                                      start_condition=first_open)

        self.motion_goals.add_grasp_bar_offset(name='pre grasp bar',
                                               root_link=root_link,
                                               tip_link=tip_link,
                                               tip_grasp_axis=tip_grasp_axis_bar,
                                               bar_center=bar_center,
                                               bar_axis=bar_axis,
                                               bar_length=bar_length,
                                               handle_link=handle_frame_id,
                                               grasp_axis_offset=grasp_axis_offset_pre,
                                               start_condition=first_open,
                                               end_condition=local_min_pre_grasp)

        bar_grasped_force = self.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.DOOR.value,
                                                           start_condition=local_min_pre_grasp)

        self.motion_goals.add_grasp_bar_offset(name='grasp bar',
                                               root_link=root_link,
                                               tip_link=tip_link,
                                               tip_grasp_axis=tip_grasp_axis_bar,
                                               bar_center=bar_center,
                                               bar_axis=bar_axis,
                                               bar_length=bar_length,
                                               handle_link=handle_frame_id,
                                               reference_linear_velocity=0.02,
                                               reference_angular_velocity=0.05,
                                               grasp_axis_offset=grasp_axis_offset,
                                               start_condition=local_min_pre_grasp,
                                               end_condition=bar_grasped_force)

        self.motion_goals.add_align_planes(tip_link=tip_link,
                                           tip_normal=x_gripper,
                                           goal_normal=x_goal,
                                           root_link=root_link,
                                           start_condition=local_min_pre_grasp,
                                           end_condition=bar_grasped_force)

        goal_point = PointStamped()
        goal_point.header.frame_id = 'base_link'

        handle_retract_direction = Vector3Stamped()
        handle_retract_direction.header.frame_id = handle_frame_id
        handle_retract_direction.vector.x = after_force_retract

        base_retract = tf.transform_vector(goal_point.header.frame_id, handle_retract_direction)

        goal_point.point = Point(base_retract.vector.x, base_retract.vector.y, base_retract.vector.z)

        grasped = self.monitors.add_local_minimum_reached(name='grasped monitor',
                                                          start_condition=bar_grasped_force)

        retracted = self.monitors.add_cartesian_position(root_link='map', tip_link='base_link', goal_point=goal_point,
                                                         start_condition=bar_grasped_force)

        self.motion_goals.add_cartesian_position_straight(root_link='map', tip_link='base_link',
                                                          goal_point=goal_point,
                                                          name='retract after hit',
                                                          start_condition=bar_grasped_force,
                                                          end_condition=retracted)

        first_close = self.monitors.add_close_hsr_gripper(name='first close gripper', start_condition=retracted)

        half_open_joint = self.monitors.add_joint_position(name='half open joint',
                                                           goal_state={hinge_joint: goal_angle_half},
                                                           threshold=0.02,
                                                           start_condition=first_close)

        self.motion_goals.add_open_container(name='half open',
                                             tip_link=tip_link,
                                             environment_link=handle_frame_id,
                                             goal_joint_state=goal_angle_half,
                                             start_condition=first_close,
                                             end_condition=half_open_joint)

        final_open = self.monitors.add_open_hsr_gripper(name='final open gripper',
                                                        start_condition=half_open_joint)

        around_local_min = self.monitors.add_local_minimum_reached(name='around door local min',
                                                                   start_condition=final_open)

        self.motion_goals.hsrb_dishwasher_door_around(handle_name=handle_frame_id,
                                                      tip_gripper_axis=tip_grasp_axis_push,
                                                      root_link=root_link,
                                                      tip_link=tip_link,
                                                      goal_angle=goal_angle_half,
                                                      start_condition=final_open,
                                                      end_condition=around_local_min)

        final_close = self.monitors.add_close_hsr_gripper(name='final close gripper',
                                                          start_condition=around_local_min)

        js_wrist = {'wrist_flex_joint': -1.3}
        joint_monitor = self.monitors.add_joint_position(goal_state=js_wrist, start_condition=final_close)
        self.motion_goals.add_joint_position(goal_state=js_wrist, start_condition=final_close,
                                             end_condition=joint_monitor)

        align_push_door = self.motion_goals.add_align_to_push_door(root_link=root_link,
                                                                   tip_link=tip_link,
                                                                   door_handle=handle_frame_id,
                                                                   door_object=door_hinge_frame_id,
                                                                   tip_gripper_axis=tip_grasp_axis_push,
                                                                   weight=WEIGHT_ABOVE_CA,
                                                                   goal_angle=goal_angle_half,
                                                                   intermediate_point_scale=1.05,
                                                                   start_condition=joint_monitor)
        self.motion_goals.update_end_condition(node_name=align_push_door, condition=align_push_door)

        js_wrist_pre_push = {'wrist_flex_joint': -1.5}
        joint_monitor_pre_push = self.monitors.add_joint_position(goal_state=js_wrist_pre_push,
                                                                  name='js pre push')
        self.monitors.update_start_condition(node_name=joint_monitor_pre_push, condition=align_push_door)

        pre_push_joint_position = self.motion_goals.add_joint_position(goal_state=js_wrist_pre_push,
                                                                       end_condition=joint_monitor_pre_push,
                                                                       name='js pre push goal')
        self.motion_goals.update_start_condition(node_name=pre_push_joint_position, condition=align_push_door)

        pre_push = self.motion_goals.add_pre_push_door(root_link=root_link,
                                                       tip_link=tip_link,
                                                       door_handle=handle_frame_id,
                                                       weight=WEIGHT_ABOVE_CA,
                                                       door_object=door_hinge_frame_id,
                                                       start_condition=joint_monitor_pre_push)
        self.motion_goals.update_end_condition(node_name=pre_push, condition=pre_push)

        full_open_joint = self.monitors.add_joint_position(name='full open joint',
                                                           goal_state={hinge_joint: goal_angle_full},
                                                           threshold=0.02)
        self.monitors.update_start_condition(node_name=full_open_joint, condition=pre_push)

        full_open_motion = self.motion_goals.add_open_container(name='full open',
                                                                tip_link=tip_link,
                                                                environment_link=handle_frame_id,
                                                                goal_joint_state=goal_angle_full,
                                                                end_condition=full_open_joint)
        self.update_start_condition(node_name=full_open_motion, condition=pre_push)

        park_joint_monitor = self.monitors.add_joint_position(name='park joint pos',
                                                              goal_state={'head_pan_joint': 0.0,
                                                                          'head_tilt_joint': 0.0,
                                                                          'arm_lift_joint': 0.0,
                                                                          'arm_flex_joint': 0.0,
                                                                          'arm_roll_joint': -1.5,
                                                                          'wrist_flex_joint': -1.5,
                                                                          'wrist_roll_joint': 0.0},
                                                              threshold=0.05,
                                                              start_condition=full_open_joint)

        self.motion_goals.add_take_pose(pose_keyword='park', start_condition=full_open_joint,
                                        end_condition=park_joint_monitor)

        set_seed = self.monitors.add_set_seed_configuration(seed_configuration={hinge_joint: 1.6},
                                                            start_condition=park_joint_monitor)
        self.monitors.add_end_motion(start_condition=set_seed)

        self.motion_goals.avoid_all_collisions(end_condition=align_push_door)
        self.motion_goals.allow_collision(env_name, allow_collision_group, start_condition=align_push_door)
        self.execute()

    def hsrb_door_opening_ft(self, handle_name: str = "iai_kitchen/iai_kitchen:arena:door_handle_inside"):
        """
        Door opening for given handle using ft-sensor.
        Newer implementation at hsr_door_opening
        """
        tip = 'hand_gripper_tool_frame'
        handle_length = 0.13
        ref_speed = 0.3
        handle_retract_distance = 0.09
        pre_grasp_distance = 0.15
        grasp_into_distance = -0.1
        ft_timeout = 10
        handle_turn_limit = 0.35
        hinge_turn_limit = -0.8

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_name
        bar_axis.vector = Vector3(0, 1, 0)

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip
        tip_grasp_axis.vector = Vector3(1, 0, 0)

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_name
        bar_center.point.y = 0.045

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.z = -1

        pre_grasp = self.monitors.add_local_minimum_reached(name='pre grasp local min')

        offset_pre = Vector3Stamped()
        offset_pre.header.frame_id = tip
        offset_pre.vector.y = pre_grasp_distance

        self.motion_goals.hsrb_door_handle_grasp(name='pre grasp', handle_name=handle_name,
                                                 handle_bar_length=handle_length,
                                                 grasp_axis_offset=offset_pre, end_condition=pre_grasp)

        open_gripper = self.monitors.add_open_hsr_gripper(start_condition=pre_grasp)

        self.motion_goals.add_align_planes(name='pre grasp align',
                                           tip_link=tip,
                                           tip_normal=x_gripper,
                                           goal_normal=x_goal,
                                           root_link='map',
                                           end_condition=open_gripper)

        self.motion_goals.add_align_planes(name='grasp align',
                                           tip_link=tip,
                                           tip_normal=x_gripper,
                                           goal_normal=x_goal,
                                           root_link='map',
                                           start_condition=open_gripper)

        offset = Vector3Stamped()
        offset.header.frame_id = tip
        offset.vector.y = grasp_into_distance

        slep = self.monitors.add_sleep(seconds=ft_timeout, start_condition=open_gripper)
        force = self.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.DOOR.value, object_type='',
                                               start_condition=open_gripper)
        self.motion_goals.hsrb_door_handle_grasp(name='grasp', handle_name=handle_name, handle_bar_length=handle_length,
                                                 grasp_axis_offset=offset, ref_speed=ref_speed,
                                                 start_condition=open_gripper,
                                                 end_condition=force)

        goal_point = PointStamped()
        goal_point.header.frame_id = 'base_link'

        handle_retract_direction = Vector3Stamped()
        handle_retract_direction.header.frame_id = handle_name
        handle_retract_direction.vector.z = handle_retract_distance

        base_retract = tf.transform_vector(goal_point.header.frame_id, handle_retract_direction)

        goal_point.point = Point(base_retract.vector.x, base_retract.vector.y, base_retract.vector.z)

        self.motion_goals.add_cartesian_position_straight(root_link='map', tip_link='base_link',
                                                          goal_point=goal_point, start_condition=force)
        grasped = self.monitors.add_local_minimum_reached(name='grasped monitor', start_condition=force)

        self.monitors.add_end_motion(start_condition=grasped)
        self.monitors.add_cancel_motion(f'not {force} and {slep} ',
                                        ObjectForceTorqueThresholdException('Door not touched!'))

        self.motion_goals.allow_all_collisions()
        self.execute()

        close_gripper = self.monitors.add_close_hsr_gripper()

        self.motion_goals.hsrb_open_door_goal(handle_name=handle_name, handle_limit=handle_turn_limit,
                                              hinge_limit=hinge_turn_limit,
                                              start_condition=close_gripper)

        self.motion_goals.allow_all_collisions()
        self.execute()

    def billy_shelf_open(self,
                         setup_pose: PoseStamped,
                         handle_link_right: str = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_right:handle',
                         hinge_joint_right: str = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_right:joint',
                         handle_link_left: str = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_left:handle',
                         door_link_left: str = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_left',
                         hinge_joint_left: str = 'iai_kitchen/shelf_billy:shelf_billy:shelf_door_left:joint',
                         vertical_grasp: bool = False,
                         open_right: bool = False,
                         open_left: bool = True,
                         simulation: bool = False):
        """
        Shelf opening implementation for given doors
        """

        if open_right:
            self.billy_shelf_right_door(handle_link_right, hinge_joint_right, vertical_grasp)
        else:
            self.motion_goals.add_joint_position(goal_state={hinge_joint_right: 1.7})
            self.add_default_end_motion_conditions()
            self.motion_goals.allow_all_collisions()
            self.execute()

        if open_left and open_right:
            # Reset Position
            self.motion_goals.add_cartesian_pose(root_link='map', tip_link='base_footprint', goal_pose=setup_pose)
            self.motion_goals.avoid_all_collisions()
            odom = self.monitors.add_local_minimum_reached()
            self.monitors.add_end_motion(start_condition=odom)
            self.execute()

        if open_left:
            self.motion_goals.add_take_pose(pose_keyword=TakePoseTypes.PARK_LEFT.value)
            self.add_default_end_motion_conditions()
            self.execute()

            self.billy_shelf_left_door_open(door_link_left, handle_link_left, hinge_joint_left, simulation)

    def billy_shelf_left_door_open(self, door_link_left, handle_link_left, hinge_joint_left, simulation):
        """
        Opening of left shelf door
        """
        if simulation:
            js = {'hand_motor_joint': 1.23}
            self.motion_goals.add_joint_position(js)
            joint_monitor = self.monitors.add_joint_position(js)
            self.motion_goals.allow_all_collisions()
            self.monitors.add_end_motion(start_condition=joint_monitor)
            self.execute()
        # Left door opening
        self.pre_pose_shelf_open(offset_x=-0.30,
                                 offset_y=-0.01,
                                 offset_z=0.03,
                                 left_door=door_link_left,
                                 left_handle=handle_link_left)
        self.execute()
        goal_states = {
            'head_pan_joint': 0,
            'head_tilt_joint': 0,
            'arm_lift_joint': 0,
            'arm_flex_joint': 0,
            'arm_roll_joint': 0,
            'wrist_flex_joint': 0,
            'wrist_roll_joint': 0
        }
        goal_point = PointStamped()
        goal_point.header.frame_id = 'hand_gripper_tool_frame'
        goal_point.point.z = 0.1
        retract_point = PointStamped()
        retract_point.header.frame_id = 'hand_gripper_tool_frame'
        retract_point.point.z = -0.02
        ft_mon_left = self.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.SHELF_GRASP.value)
        cart_mon = self.monitors.add_cartesian_position(root_link='map',
                                                        tip_link='hand_gripper_tool_frame',
                                                        goal_point=retract_point,
                                                        start_condition=ft_mon_left)
        self.motion_goals.add_joint_position_stop(goal_state=goal_states, end_condition=cart_mon)
        self.motion_goals.add_cartesian_position(name='ft push',
                                                 root_link='map',
                                                 tip_link='hand_gripper_tool_frame',
                                                 goal_point=goal_point,
                                                 end_condition=ft_mon_left)
        self.motion_goals.add_cartesian_position(name='ft retract',
                                                 root_link='map',
                                                 tip_link='hand_gripper_tool_frame',
                                                 goal_point=retract_point,
                                                 start_condition=ft_mon_left,
                                                 end_condition=cart_mon)
        close_gripper = self.monitors.add_close_hsr_gripper(start_condition=cart_mon)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_footprint'
        goal_pose.pose.orientation.w = 1
        self.motion_goals.add_cartesian_pose(goal_pose=goal_pose,
                                             root_link='map',
                                             tip_link='base_footprint',
                                             start_condition=cart_mon)
        self.monitors.add_end_motion(start_condition=close_gripper)
        self.execute()

        if simulation:
            js = {'hand_motor_joint': 0}
            self.motion_goals.add_joint_position(js)
            joint_monitor = self.monitors.add_joint_position(js)
            self.motion_goals.allow_all_collisions()
            self.monitors.add_end_motion(start_condition=joint_monitor)
            self.execute()

        self.open_shelf_door(left_door_hinge=hinge_joint_left,
                             left_door=door_link_left,
                             left_handle=handle_link_left)
        self.execute()

        open_gripper = self.monitors.add_open_hsr_gripper()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_footprint'
        goal_pose.pose.orientation.w = 1
        self.motion_goals.add_cartesian_pose(goal_pose=goal_pose, root_link='map', tip_link='base_footprint')
        self.monitors.add_end_motion(start_condition=open_gripper)
        self.execute()
        if simulation:
            js = {'hand_motor_joint': 1.23}
            self.motion_goals.add_joint_position(js)
            joint_monitor = self.monitors.add_joint_position(js)
            self.motion_goals.allow_all_collisions()
            self.monitors.add_end_motion(start_condition=joint_monitor)
            self.execute()

    def billy_shelf_right_door(self, handle_link_right, hinge_joint_right, vertical_grasp):
        """
        Opening of right shelf door
        """
        bar_center = PointStamped()
        bar_center.header.frame_id = handle_link_right
        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_link_right
        bar_axis.vector.z = 1
        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = 'hand_gripper_tool_frame'
        tip_grasp_axis.vector.z = 1
        align_gripper = Vector3Stamped()
        align_gripper.header.frame_id = 'hand_gripper_tool_frame'
        align_goal = Vector3Stamped()
        align_goal.header.frame_id = handle_link_right
        if vertical_grasp:
            align_gripper.vector.y = -1
            align_goal.vector.x = 1
        else:
            align_gripper.vector.x = -1
            align_goal.vector.x = 1
        self.motion_goals.add_align_planes(name='pre grasp align z',
                                           tip_link='hand_gripper_tool_frame',
                                           tip_normal=align_gripper,
                                           goal_normal=align_goal,
                                           root_link='map')
        pre_grasp_offset = Vector3Stamped()
        pre_grasp_offset.header.frame_id = handle_link_right
        pre_grasp_offset.vector.z = -0.05
        ft_offset = Vector3Stamped()
        ft_offset.header.frame_id = handle_link_right
        ft_offset.vector.z = 0.1
        self.monitors.add_open_hsr_gripper(name='first open gripper')
        local_min_pre_grasp = self.monitors.add_local_minimum_reached(name='pre grasp local min')
        self.motion_goals.add_grasp_bar_offset(name='pre grasp bar',
                                               root_link='map',
                                               tip_link='hand_gripper_tool_frame',
                                               tip_grasp_axis=tip_grasp_axis,
                                               bar_center=bar_center,
                                               bar_axis=bar_axis,
                                               bar_length=0.001,
                                               grasp_axis_offset=pre_grasp_offset,
                                               handle_link=handle_link_right,
                                               end_condition=local_min_pre_grasp)
        ft_monitor = self.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.DOOR.value,
                                                    start_condition=local_min_pre_grasp)
        self.motion_goals.add_grasp_bar_offset(name='grasp bar offset',
                                               root_link='map',
                                               tip_link='hand_gripper_tool_frame',
                                               tip_grasp_axis=tip_grasp_axis,
                                               bar_center=bar_center,
                                               bar_axis=bar_axis,
                                               bar_length=0.001,
                                               grasp_axis_offset=ft_offset,
                                               handle_link=handle_link_right,
                                               reference_linear_velocity=0.01,
                                               reference_angular_velocity=0.05,
                                               start_condition=local_min_pre_grasp,
                                               end_condition=ft_monitor)
        goal_point = PointStamped()
        goal_point.header.frame_id = 'base_link'
        handle_retract_direction = Vector3Stamped()
        handle_retract_direction.header.frame_id = handle_link_right
        handle_retract_direction.vector.z = -0.06
        base_retract = tf.transform_vector(goal_point.header.frame_id, handle_retract_direction)
        goal_point.point = Point(base_retract.vector.x, base_retract.vector.y, base_retract.vector.z)
        grasped = self.monitors.add_cartesian_position(root_link='map', tip_link='base_link', goal_point=goal_point,
                                                       name='grasped monitor', start_condition=ft_monitor)
        close_gripper = self.monitors.add_close_hsr_gripper(start_condition=grasped, name='first close gripper')
        self.motion_goals.add_cartesian_position(root_link='map', tip_link='base_link',
                                                 goal_point=goal_point, start_condition=ft_monitor,
                                                 reference_velocity=0.02, end_condition=grasped)
        align_goal = Vector3Stamped()
        align_goal.header.frame_id = handle_link_right
        align_goal.vector.z = -1
        x_base = Vector3Stamped()
        x_base.header.frame_id = 'base_link'
        x_base.vector.y = 1
        door_open = self.monitors.add_joint_position(goal_state={hinge_joint_right: 1.65},
                                                     start_condition=close_gripper)
        open_gripper = self.monitors.add_open_hsr_gripper(start_condition=door_open, name='second open gripper')
        self.motion_goals.add_align_planes(goal_normal=align_goal, tip_link='base_link', tip_normal=x_base,
                                           root_link='map',
                                           start_condition=close_gripper,
                                           end_condition=door_open)
        self.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame', environment_link=handle_link_right,
                                             start_condition=close_gripper,
                                             end_condition=door_open)
        self.monitors.add_end_motion(start_condition=open_gripper)
        self.execute()

    def hsr_door_opening(self,
                         handle_name: str = "iai_kitchen/iai_kitchen:arena:door_handle_inside",
                         hinge_joint: str = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint",
                         handle_joint: str = "iai_kitchen/iai_kitchen:arena:door_handle_joint",
                         hinge_name: str = 'iai_kitchen/iai_kitchen:arena:door_center',
                         tip_link: str = 'hand_gripper_tool_frame',
                         handle_length: float = 0.01,
                         ref_speed: float = 0.3,
                         handle_retract_distance: float = 0.063,
                         bar_center_offset: float = 0.02,
                         pre_grasp_distance: float = 0.15,
                         grasp_into_distance: float = -0.1,
                         ft_timeout: float = 10):
        """
        Opening of arena door
        """

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_name
        bar_axis.vector = Vector3(0, 1, 0)

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip_link
        tip_grasp_axis.vector = Vector3(1, 0, 0)

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_name

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = tip_link
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.z = -1

        pre_grasp = self.monitors.add_local_minimum_reached(name='pre grasp local min')

        offset_pre = Vector3Stamped()
        offset_pre.header.frame_id = tip_link
        offset_pre.vector.y = pre_grasp_distance
        offset_pre.vector.z = bar_center_offset

        self.motion_goals.add_joint_position(goal_state={hinge_joint: 0})

        self.motion_goals.hsrb_door_handle_grasp(name='pre grasp', handle_name=handle_name,
                                                 handle_bar_length=handle_length,
                                                 grasp_axis_offset=offset_pre, end_condition=pre_grasp)

        open_gripper = self.monitors.add_open_hsr_gripper(start_condition=pre_grasp)

        self.motion_goals.add_align_planes(name='pre grasp align',
                                           tip_link=tip_link,
                                           tip_normal=x_gripper,
                                           goal_normal=x_goal,
                                           root_link='map',
                                           end_condition=open_gripper)

        self.motion_goals.add_align_planes(name='grasp align',
                                           tip_link=tip_link,
                                           tip_normal=x_gripper,
                                           goal_normal=x_goal,
                                           root_link='map',
                                           start_condition=open_gripper)

        offset = Vector3Stamped()
        offset.header.frame_id = tip_link
        offset.vector.y = grasp_into_distance
        offset.vector.z = bar_center_offset

        slep = self.monitors.add_sleep(seconds=ft_timeout, start_condition=open_gripper)
        force = self.monitors.add_force_torque(threshold_enum=ForceTorqueThresholds.DOOR.value, object_type='',
                                               start_condition=open_gripper)
        self.motion_goals.hsrb_door_handle_grasp(name='grasp', handle_name=handle_name, handle_bar_length=handle_length,
                                                 grasp_axis_offset=offset, ref_speed=ref_speed,
                                                 start_condition=open_gripper,
                                                 end_condition=force)

        goal_point = PointStamped()
        goal_point.header.frame_id = 'base_link'

        handle_retract_direction = Vector3Stamped()
        handle_retract_direction.header.frame_id = handle_name
        handle_retract_direction.vector.z = handle_retract_distance

        base_retract = tf.transform_vector(goal_point.header.frame_id, handle_retract_direction)

        goal_point.point = Point(base_retract.vector.x, base_retract.vector.y, base_retract.vector.z)

        self.motion_goals.add_cartesian_position(root_link='map', tip_link='base_link', reference_velocity=0.05,
                                                 goal_point=goal_point, start_condition=force)
        grasped = self.monitors.add_cartesian_position(root_link='map', tip_link='base_link', goal_point=goal_point,
                                                       name='grasped monitor', start_condition=force)

        close_gripper = self.monitors.add_close_hsr_gripper(start_condition=grasped)
        sleep = self.monitors.add_sleep(name='settle grasp', seconds=0.5, start_condition=close_gripper)

        self.monitors.add_end_motion(start_condition=close_gripper)
        self.monitors.add_cancel_motion(f'not {force} and {slep} ',
                                        ObjectForceTorqueThresholdException('Door not touched!'))

        self.motion_goals.allow_all_collisions()
        self.execute()

        self.motion_goals.add_joint_position(goal_state={hinge_joint: 0})
        self.motion_goals.add_open_container(tip_link='hand_gripper_tool_frame', environment_link=handle_name,
                                             goal_joint_state=0.35)
        handle_monitor = self.monitors.add_joint_position(goal_state={handle_joint: 0.35})
        self.motion_goals.allow_all_collisions()
        self.monitors.add_end_motion(start_condition=handle_monitor)

        self.execute()

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.z = -1

        x_base = Vector3Stamped()
        x_base.header.frame_id = 'base_link'
        x_base.vector.y = 1

        self.motion_goals.add_align_planes(goal_normal=x_goal, tip_link='base_link', tip_normal=x_base, root_link='map')
        goal_states = {
            'head_pan_joint': 0,
            'head_tilt_joint': 0,
            'arm_lift_joint': 0,
            'arm_flex_joint': 0,
            'arm_roll_joint': 0,
            'wrist_flex_joint': 0,
            'wrist_roll_joint': 0
        }
        self.motion_goals.add_joint_position_stop(goal_state=goal_states)
        self.motion_goals.add_close_container(tip_link='hand_gripper_tool_frame', environment_link=hinge_name)
        door_hinge_monitor = self.monitors.add_joint_position(goal_state={hinge_joint: -1.3})
        open_gripper = self.monitors.add_open_hsr_gripper(start_condition=door_hinge_monitor)
        self.monitors.add_end_motion(start_condition=open_gripper)
        self.motion_goals.allow_all_collisions()

        self.execute()

    def door_handle_grasping(self,
                             handle_name: str,
                             hinge_joint: str,
                             handle_retract_distance: float,
                             tip_link: str = 'hand_gripper_tool_frame',
                             ref_speed: float = 0.5,
                             grasp_into_distance: float = -0.15,
                             pre_grasp_distance: float = 0.2,
                             offset_along_handle: float = 0.03,
                             ft_timeout: float = 10.0,
                             camera_link: Optional[str] = None):
        """
        Door handle grasping with force-torque-sensor and optionally hand camera. Preparation for opening door

        :param handle_name: frame id of the door handle
        :param hinge_joint: frame id of the door hinge
        :param handle_retract_distance: distance gripper retracts after collision with door handle
        :param tip_link: tip link of the kin chain
        :param ref_speed: reference speed for approaching the handle for forcetorque detection
        :param pre_grasp_distance: distance in front of the handle, from where the ft movement starts
        :param grasp_into_distance: distance from handle position how far further the gripper can go
        :param offset_along_handle: offset from center of handle for better grasping position
        :param ft_timeout: time in seconds after which the ft throws an error when no collision is detected
        :param camera_link: Optional camera link for visual servoing approach with RoboKudo
        """

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_name
        bar_axis.vector = Vector3(0, 1, 0)

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip_link
        tip_grasp_axis.vector = Vector3(1, 0, 0)

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_name

        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = tip_link
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_name
        x_goal.vector.z = -1

        grasp_axis_offset = Vector3Stamped()
        grasp_axis_offset.header.frame_id = handle_name
        grasp_axis_offset.vector.z = grasp_into_distance

        pre_grasp_axis_offset = Vector3Stamped()
        pre_grasp_axis_offset.header.frame_id = handle_name
        pre_grasp_axis_offset.vector.z = pre_grasp_distance

        handle_retract = PointStamped()
        handle_retract.header.frame_id = tip_link
        handle_retract.point.z = handle_retract_distance

        grasp_push = PointStamped()
        grasp_push.header.frame_id = tip_link
        grasp_push.point.z = pre_grasp_distance - grasp_into_distance

        js = {
            'head_pan_joint': 0,
            'head_tilt_joint': 0,
            'arm_flex_joint': 0,
            'arm_roll_joint': 0,
            'wrist_flex_joint': -np.pi / 2,
            'wrist_roll_joint': -np.pi / 2
        }
        jps = self.motion_goals.add_joint_position(goal_state=js,
                                                   name='hold fixed grasping position',
                                                   threshold=0.05)

        open_gripper = self.monitors.add_open_hsr_gripper(start_condition=jps)

        handle_correction_offset = PointStamped()
        handle_correction_offset.header.frame_id = tip_link
        handle_correction_offset.point.x = offset_along_handle

        grasp = self.motion_goals.add_grasp_with_ft_sensor(root_link='map',
                                                           tip_link=tip_link,
                                                           handle_name=handle_name,
                                                           tip_grasp_axis=tip_grasp_axis,
                                                           bar_axis=bar_axis,
                                                           tip_retract=handle_retract,
                                                           handle_align_axis=x_goal,
                                                           tip_align_axis=x_gripper,
                                                           grasp_axis_offset=grasp_axis_offset,
                                                           pre_grasp_axis_offset=pre_grasp_axis_offset,
                                                           hinge_joint=hinge_joint,
                                                           timeout=ft_timeout,
                                                           ft_grasp_ref_speed=ref_speed,
                                                           camera_link=camera_link,
                                                           tip_push=grasp_push,
                                                           handle_correction_offset=handle_correction_offset,
                                                           start_condition=open_gripper)
        self.update_end_condition(node_name=grasp, condition=grasp)

        close_gripper = self.monitors.add_close_hsr_gripper(start_condition=grasp)

        self.monitors.add_end_motion(start_condition=close_gripper)

        self.motion_goals.allow_all_collisions()
        self.execute()

    # TODO: when collision avoidance is working with changes to it during runtime -> make one unified statechart/motion and not in parts
    def door_opening_with_moving_around(self,
                                        root_link: str,
                                        tip_link: str,
                                        handle_name: str,
                                        door_handle_for_hinge: str,
                                        door_center: str,
                                        handle_retract_distance: float = -0.15,
                                        handle_turn_limit: float = 0.4,
                                        full_hinge_turn_limit: float = -1.4,
                                        pre_push_hinge_turn_limit: float = -0.5,
                                        height_offset: float = -0.15):
        """
        Door opening with moving around the door to open it from the inside

        :param root_link: root link of the kin chain (e.g. map)
        :param tip_link: tip_link of the kin chain (e.g. hand_gripper_tool_frame)
        :param handle_name: frame id of the door handle (e.g. iai_kitchen/iai_kitchen:arena:door_handle_inside)
        :param door_handle_for_hinge: Handle frame id for the hinge movement (e.g. iai_kitchen/iai_kitchen:arena:door_handle_link)
        :param door_center: frame id of door center used for opening door fully
        :param handle_retract_distance: distance the gripper retracts after partly opening the door to be able to push
        :param handle_turn_limit: how much the handle is to be turned
        :param full_hinge_turn_limit: how far the door is to be opened in the end
        :param pre_push_hinge_turn_limit: how far the door is opened before the pushing sequence is used
        :param height_offset: height offset for reaching around door handle to not collide with the handle
        """
        tip = tip_link
        root = root_link
        open_door_name = 'OpenDoorGoal'

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = tip
        tip_grasp_axis.vector = Vector3(1, 0, 0)

        move_around_name = 'MovingAroundAtTheSpeedOfSound'
        multipliers = [(7 / 5, -0.3, 'down_long'),
                       (7 / 5, 0.4, 'up_long'),
                       (0.8, 0.7, 'up_short')]

        offset = Vector3Stamped()
        offset.header.frame_id = root
        offset.vector = Vector3(0, 0, height_offset)

        retract_name = 'retract_after_ft'
        handle_retract = PointStamped()
        handle_retract.header.frame_id = tip
        handle_retract.point.z = handle_retract_distance
        goal_orientation = QuaternionStamped()
        goal_orientation.header.frame_id = tip
        goal_orientation.quaternion.w = 1

        open_goal = self.motion_goals.hsrb_open_door_goal(handle_name=handle_name,
                                                          handle_limit=handle_turn_limit,
                                                          hinge_limit=pre_push_hinge_turn_limit,
                                                          name=open_door_name,
                                                          end_condition=open_door_name)

        slep = self.monitors.add_sleep(seconds=0.5, start_condition=open_goal)

        open_gripper = self.monitors.add_open_hsr_gripper(start_condition=slep)

        self.motion_goals.add_cartesian_orientation(goal_orientation=goal_orientation,
                                                    root_link=root,
                                                    tip_link=tip,
                                                    name='fix_rotation_on_retract',
                                                    start_condition=slep,
                                                    end_condition=retract_name)

        open_goal_retract = self.motion_goals.add_cartesian_position(root_link=root,
                                                                     tip_link=tip,
                                                                     goal_point=handle_retract,
                                                                     name=retract_name,
                                                                     threshold=0.001,
                                                                     start_condition=open_gripper,
                                                                     end_condition=retract_name)

        self.motion_goals.allow_collision(group1='arm',
                                          group2='iai_kitchen')
        self.monitors.add_end_motion(open_goal_retract)
        self.execute()

        open_goal_moving_around = self.motion_goals.add_move_around_hinge(handle_name=door_handle_for_hinge,
                                                                          tip_gripper_axis=tip_grasp_axis,
                                                                          root_link=root,
                                                                          tip_link=tip,
                                                                          goal_angle=pre_push_hinge_turn_limit,
                                                                          multipliers=multipliers,
                                                                          offset=offset,
                                                                          align_gripper=MoveAroundHingeAlign.ALL,
                                                                          name=move_around_name,
                                                                          start_condition='',
                                                                          end_condition=move_around_name)

        self.motion_goals.avoid_all_collisions()
        self.monitors.add_end_motion(move_around_name)
        self.execute()

        close_gripper = self.monitors.add_close_hsr_gripper()

        pre_push = self.motion_goals.add_pre_push_door(root_link=root,
                                                       tip_link=tip,
                                                       door_handle=door_handle_for_hinge,
                                                       weight=WEIGHT_ABOVE_CA,
                                                       door_object=door_center,
                                                       start_condition=close_gripper)
        self.update_end_condition(pre_push, pre_push)

        open_full = self.motion_goals.add_open_container(tip_link=tip,
                                                         environment_link=door_center,
                                                         name='Push open door',
                                                         goal_joint_state=full_hinge_turn_limit,
                                                         start_condition=pre_push)
        self.update_end_condition(open_full, open_full)

        self.motion_goals.allow_collision(group1='arm',
                                          group2='iai_kitchen')
        self.monitors.add_end_motion(start_condition=open_full)
        self.execute()
