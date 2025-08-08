import os
from copy import deepcopy
from enum import Enum
from typing import Optional, List, Tuple

import numpy as np
from std_msgs.msg import ColorRGBA

from giskardpy import casadi_wrapper as w, casadi_wrapper as cas
from giskardpy.data_types.data_types import PrefixName
from giskardpy.data_types.exceptions import ObjectForceTorqueThresholdException
from giskardpy.data_types.suturo_types import GraspTypes, ForceTorqueThresholds, MoveAroundHingeAlign
from giskardpy.data_types.suturo_types import ObjectTypes, TakePoseTypes
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.motion_statechart.goals.cartesian_goals import CartesianPosition, CartesianOrientation
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.motion_statechart.goals.open_close import Open
from giskardpy.motion_statechart.monitors.force_torque_monitor import PayloadForceTorque
from giskardpy.motion_statechart.monitors.joint_monitors import JointGoalReached
from giskardpy.motion_statechart.monitors.monitors import Monitor, CancelMotion
from giskardpy.motion_statechart.monitors.payload_monitors import Sleep
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from giskardpy.motion_statechart.tasks.task import WEIGHT_ABOVE_CA, Task, WEIGHT_BELOW_CA
from giskardpy_ros.monitors.handle_offset_monitor import HandleOffsetCorrection

if 'GITHUB_WORKFLOW' not in os.environ:
    pass


class ContextActionModes(Enum):
    grasping = 'grasping'
    placing = 'placing'
    pouring = 'pouring'
    door_opening = 'door-opening'


class ObjectGoal(Goal):
    """
    Inherit from this class if the goal tries to get the object by name from the world
    """

    def get_object_by_name(self, object_name):
        try:
            get_middleware().loginfo('trying to get objects with name')

            object_link = god_map.world.get_link(object_name)
            object_collisions = object_link.collisions
            if len(object_collisions) == 0:
                object_geometry = BoxGeometry(link_T_geometry=np.eye(4), depth=0, width=0, height=0, color=None)
            else:
                object_geometry: LinkGeometry = object_link.collisions[0]

            goal_pose = god_map.world.compute_fk('map', god_map.world.search_for_link_name(object_name))

            get_middleware().loginfo(f'goal_pose by name: {goal_pose}')

            # Declare instance of geometry
            if isinstance(object_geometry, BoxGeometry):
                object_type = 'box'
                object_geometry: BoxGeometry = object_geometry
                # FIXME use expression instead of vector3, unless its really a vector
                object_size = cas.Vector3.from_xyz(x=object_geometry.width, y=object_geometry.depth,
                                                   z=object_geometry.height)

            elif isinstance(object_geometry, CylinderGeometry):
                object_type = 'cylinder'
                object_geometry: CylinderGeometry = object_geometry
                object_size = cas.Vector3.from_xyz(x=object_geometry.radius, y=object_geometry.radius,
                                                   z=object_geometry.height)

            elif isinstance(object_geometry, SphereGeometry):
                object_type = 'sphere'
                object_geometry: SphereGeometry = object_geometry
                object_size = cas.Vector3.from_xyz(x=object_geometry.radius, y=object_geometry.radius,
                                                   z=object_geometry.radius)

            else:
                raise Exception('Not supported geometry')

            get_middleware().loginfo(f'Got geometry: {object_type}')
            return goal_pose, object_size

        except:
            get_middleware().loginfo('Could not get geometry from name')
            return None


class Reaching(ObjectGoal):
    def __init__(self,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 grasp: str,
                 align: str,
                 name: str = None,
                 object_name: Optional[str] = None,
                 object_shape: Optional[str] = None,
                 goal_pose: Optional[cas.TransMatrix] = None,
                 object_size: Optional[cas.Vector3] = None,
                 velocity: float = 0.2,
                 weight: float = WEIGHT_ABOVE_CA):
        """
            Concludes Reaching type goals.
            Executes them depending on the given context action.
            Context is a dictionary in an action is given as well as situational parameters.
            All available context Messages are found in the Enum 'ContextTypes'

            :param grasp: Direction from which object is being grasped from
            :param align: String which decides whether HSR tries to vertically align with the object before reaching
            :param name: name of the executed goal, in this case Reaching
            :param object_name: Name of the object to use. Optional as long as goal_pose and object_size are filled instead
            :param object_shape: Shape of the object to manipulate. Edit object size when having a sphere or cylinder
            :param goal_pose: Goal pose for the object. Alternative if no object name is given.
            :param object_size: Given object size. Alternative if no object name is given.
            :param root_link: Current root Link
            :param tip_link: Current tip link
            :param velocity: Desired velocity of this goal
            :param weight: weight of this goal
        """
        if name is None:
            name = 'Reaching'

        super().__init__(name=name)

        if root_link is None:
            root_link = god_map.world.groups[god_map.world.robot_name].root_link.name.short_name
        if tip_link is None:
            tip_link = self.gripper_tool_frame

        self.grasp = grasp
        self.align = align
        self.object_name = object_name
        self.object_shape = object_shape
        self.root_link_name = root_link
        self.tip_link_name = tip_link
        self.velocity = velocity
        self.weight = weight
        self.offsets = cas.Vector3.from_xyz(0, 0, 0)
        self.careful = False
        self.object_in_world = goal_pose is None

        # Get object geometry from name
        if goal_pose is None:
            self.goal_pose, self.object_size = self.get_object_by_name(self.object_name)
            self.reference_frame = self.object_name

        else:
            try:
                god_map.world.search_for_link_name(goal_pose.reference_frame)
                self.goal_pose = goal_pose
            except:
                get_middleware().logwarn(f'Couldn\'t find {goal_pose.reference_frame}. Searching in tf.')
                self.goal_pose = god_map.world.compute_fk(god_map.world.search_for_link_name('map'), goal_pose)

            self.object_size = object_size
            self.reference_frame = 'base_footprint'
            get_middleware().logwarn(f'Warning: Object not in giskard world')

        if self.object_shape == 'sphere' or self.object_shape == 'cylinder':
            self.offsets = cas.Vector3.from_xyz(x=self.object_size.x, y=self.object_size.y, z=self.object_size.z)

        # TODO: fine tune and add correct object names
        # FIXME: add Parameter instead of hard-coded values
        elif self.object_name == 'Plate':
            self.offsets = cas.Vector3.from_xyz(x=-(self.object_size.x / 2) + 0.03, y=self.object_size.y,
                                                z=self.object_size.z)

        elif self.object_name == 'Bowl':
            self.offsets = cas.Vector3.from_xyz(x=-(self.object_size.x / 2) + 0.15, y=self.object_size.y,
                                                z=self.object_size.z)

        elif self.object_name == 'Cutlery':
            self.offsets = cas.Vector3.from_xyz(x=-(self.object_size.x / 2) + 0.02, y=self.object_size.y,
                                                z=self.object_size.z)

        # TODO: Test Tray on real robot
        elif self.object_name == ObjectTypes.OT_Tray.value:
            self.offsets = cas.Vector3.from_xyz(x=-(self.object_size.x / 3), y=self.object_size.y,
                                                z=self.object_size.z - 0.05)

        else:
            if self.object_in_world:
                self.offsets = cas.Vector3.from_xyz(-self.object_size.x / 2, self.object_size.y / 2,
                                                    self.object_size.z / 2)
            else:
                self.offsets = cas.Vector3.from_xyz(max(min(0.08, self.object_size.x.to_np() / 2), 0.05), 0, 0)

        if all(self.grasp != member.value for member in GraspTypes):
            raise Exception(f"Unknown grasp value: {grasp}")

        go = GraspObject(goal_pose=self.goal_pose,
                         reference_frame_alignment=self.reference_frame,
                         offsets=self.offsets,
                         grasp=self.grasp,
                         align=self.align,
                         root_link=self.root_link_name,
                         tip_link=self.tip_link_name,
                         velocity=self.velocity,
                         weight=self.weight,
                         name='GraspObject Reaching')
        go.start_condition = self.start_condition
        go.pause_condition = self.pause_condition
        go.end_condition = self.end_condition
        self.add_goal(go)


class GraspObject(ObjectGoal):
    def __init__(self,
                 goal_pose: cas.TransMatrix,
                 align: str,
                 grasp: str,
                 offsets: cas.Vector3 = cas.Vector3.from_xyz(0, 0, 0),
                 name: Optional[str] = None,
                 reference_frame_alignment: Optional[str] = None,
                 root_link: Optional[PrefixName] = None,
                 tip_link: Optional[PrefixName] = None,
                 velocity: float = 0.2,
                 weight: float = WEIGHT_ABOVE_CA):
        """
            Concludes Reaching type goals.
            Executes them depending on the given context action.
            Context is a dictionary in an action is given as well as situational parameters.
            All available context Messages are found in the Enum 'ContextTypes'

            :param goal_pose: Goal pose for the object.
            :param align: States if the gripper should be rotated and in which "direction"
            :param offsets: Optional parameter to pass a specific offset in x, y or z direction
            :param grasp: The Direction from with an object should be grasped
            :param name: Name of the executed goal, in this case GraspObject
            :param reference_frame_alignment: Reference frame to align with. Is usually either an object link or 'base_footprint'
            :param root_link: Current root Link
            :param tip_link: Current tip link
            :param velocity: Desired velocity of this goal
            :param weight: weight of this goal
        """
        if name is None:
            name = 'GraspObject'

        super().__init__(name=name)
        self.goal_pose = goal_pose

        self.offsets = offsets
        self.grasp = grasp
        self.align = align

        if reference_frame_alignment is None:
            reference_frame_alignment = 'base_footprint'

        if root_link is None:
            root_link = god_map.world.groups[god_map.world.robot_name].root_link.name

        if tip_link is None:
            tip_link = self.gripper_tool_frame

        self.reference_link = god_map.world.search_for_link_name(reference_frame_alignment)
        self.root_link = god_map.world.search_for_link_name(root_link)
        self.tip_link = god_map.world.search_for_link_name(tip_link)

        self.velocity = velocity
        self.weight = weight

        self.goal_frontal_axis = cas.Vector3()
        self.goal_frontal_axis.reference_frame = self.reference_link

        self.tip_frontal_axis = cas.Vector3()
        self.tip_frontal_axis.reference_frame = self.tip_link

        self.goal_vertical_axis = cas.Vector3()
        self.goal_vertical_axis.reference_frame = self.reference_link

        self.tip_vertical_axis = cas.Vector3()
        self.tip_vertical_axis.reference_frame = self.tip_link

        root_goal_point = self.goal_pose.to_position()

        self.goal_point = god_map.world.transform(self.reference_link, root_goal_point)

        # TODO: Refactor so that grasping from above can be used with Tray
        if self.grasp == GraspTypes.ABOVE.value:
            self.goal_vertical_axis.x = self.standard_forward.x
            self.goal_vertical_axis.y = self.standard_forward.y
            self.goal_vertical_axis.z = self.standard_forward.z
            v = multiply_vector(self.standard_up, -1)
            self.goal_frontal_axis.x = v.x
            self.goal_frontal_axis.y = v.y
            self.goal_frontal_axis.z = v.z
            self.goal_point.z += self.offsets.z

        elif self.grasp == GraspTypes.BELOW.value:
            v = multiply_vector(self.standard_forward, -1)
            self.goal_vertical_axis.x = v.x
            self.goal_vertical_axis.y = v.y
            self.goal_vertical_axis.z = v.z
            self.goal_frontal_axis.x = self.standard_up.x
            self.goal_frontal_axis.y = self.standard_up.y
            self.goal_frontal_axis.z = self.standard_up.z
            self.goal_point.z -= self.offsets.z

        elif self.grasp == GraspTypes.FRONT.value:
            # v = cas.Vector3.from_xyz(0, 2, 0)
            self.goal_vertical_axis.x = self.standard_up.x
            self.goal_vertical_axis.y = self.standard_up.y  # v.y
            self.goal_vertical_axis.z = self.standard_up.z
            self.goal_frontal_axis.x = self.standard_forward.x  # self.standard_up.x
            self.goal_frontal_axis.y = self.standard_forward.y  # self.standard_up.y
            self.goal_frontal_axis.z = self.standard_forward.z  # self.standard_up.z

            self.goal_point.z -= 0.01
        # self.goal_point.x += self.offsets.x

        elif self.grasp == GraspTypes.LEFT.value:
            self.goal_vertical_axis.x = self.standard_up.x
            self.goal_vertical_axis.y = self.standard_up.y
            self.goal_vertical_axis.z = self.standard_up.z
            self.goal_frontal_axis.x = self.gripper_left.x
            self.goal_frontal_axis.y = self.gripper_left.y
            self.goal_frontal_axis.z = self.gripper_left.z

        elif self.grasp == GraspTypes.RIGHT.value:
            v = multiply_vector(self.gripper_left, -1)
            self.goal_vertical_axis.x = self.standard_up.x
            self.goal_vertical_axis.y = self.standard_up.y
            self.goal_vertical_axis.z = self.standard_up.z
            self.goal_frontal_axis.x = v.x
            self.goal_frontal_axis.y = v.y
            self.goal_frontal_axis.z = v.z

        if self.align == "vertical":
            self.tip_vertical_axis.x = self.gripper_left.x
            self.tip_vertical_axis.y = self.gripper_left.y
            self.tip_vertical_axis.z = self.gripper_left.z

        else:
            self.tip_vertical_axis.x = self.gripper_up.x
            self.tip_vertical_axis.y = self.gripper_up.y
            self.tip_vertical_axis.z = self.gripper_up.z

        self.tip_frontal_axis.x = self.gripper_forward.x
        self.tip_frontal_axis.y = self.gripper_forward.y
        self.tip_frontal_axis.z = self.gripper_forward.z

        # Position
        cp = CartesianPosition(root_link=self.root_link,
                               tip_link=self.tip_link,
                               goal_point=self.goal_point,
                               reference_velocity=self.velocity,
                               weight=self.weight)
        cp.start_condition = self.start_condition
        cp.pause_condition = self.pause_condition
        cp.end_condition = self.end_condition
        self.add_task(cp)

        # Align vertical
        apv = AlignPlanes(name='APlanesVertical',
                          root_link=self.root_link,
                          tip_link=self.tip_link,
                          goal_normal=self.goal_vertical_axis,
                          tip_normal=self.tip_vertical_axis,
                          reference_velocity=self.velocity,
                          weight=self.weight)
        apv.start_condition = self.start_condition
        apv.pause_condition = self.pause_condition
        apv.end_condition = self.end_condition
        self.add_task(apv)

        # Align frontal
        aph = AlignPlanes(name='APlanesFrontal',
                          root_link=self.root_link,
                          tip_link=self.tip_link,
                          goal_normal=self.goal_frontal_axis,
                          tip_normal=self.tip_frontal_axis,
                          reference_velocity=self.velocity,
                          weight=self.weight)
        aph.start_condition = self.start_condition
        aph.pause_condition = self.pause_condition
        aph.end_condition = self.end_condition
        self.add_task(aph)

        kr = KeepRotationGoal(tip_link='base_footprint',
                              weight=self.weight)
        kr.start_condition = self.start_condition
        kr.pause_condition = self.pause_condition
        kr.end_condition = self.end_condition
        self.add_constraints_of_goal(kr)


class VerticalMotion(ObjectGoal):
    def __init__(self,
                 action: str = None,
                 name: str = None,
                 distance: float = 0.02,
                 root_link: Optional[str] = None,
                 tip_link: Optional[str] = None,
                 velocity: float = 0.2,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Move the tip link vertical according to the given context.

        :param action: Action to take.
        :param name: Name of the goal, in this case VerticalMotion.
        :param distance: Optional parameter to adjust the distance to move.
        :param root_link: Current root Link
        :param tip_link: Current tip link
        :param velocity: Desired velocity of this goal
        :param weight: weight of this goal
        """
        if name is None:
            name = 'VerticalMotion'

        super().__init__(name=name)

        if root_link is None:
            root_link = 'base_footprint'
        if tip_link is None:
            tip_link = self.gripper_tool_frame
        self.distance = distance
        self.root_link = god_map.world.search_for_link_name(root_link)
        self.tip_link = god_map.world.search_for_link_name(tip_link)
        self.velocity = velocity
        self.weight = weight
        self.base_footprint = god_map.world.search_for_link_name('base_footprint')
        self.action = action

        start_point_tip = cas.TransMatrix()
        start_point_tip.reference_frame = self.tip_link
        goal_point_base = god_map.world.transform(self.base_footprint, start_point_tip)

        up = ContextActionModes.grasping.value in self.action
        down = ContextActionModes.placing.value in self.action
        if up:
            goal_point_base.z += self.distance
        elif down:
            goal_point_base.z -= self.distance
        else:
            get_middleware().logwarn('no direction given')

        krg = KeepRotationGoal(tip_link=self.tip_link.short_name,
                               weight=self.weight)
        krg.start_condition = self.start_condition
        krg.pause_condition = self.pause_condition
        krg.end_condition = self.end_condition
        self.add_constraints_of_goal(krg)

        goal_point_tip = god_map.world.transform(self.tip_link, goal_point_base)
        self.goal_point = deepcopy(goal_point_tip)
        # self.root_T_tip_start = god_map.world.compute_fk_np(self.root_link, self.tip_link)
        # self.start_tip_T_current_tip = np.eye(4)

        # start_tip_T_current_tip = w.TransMatrix(self.get_parameter_as_symbolic_expression('start_tip_T_current_tip'))
        root_T_tip = god_map.world.compose_fk_expression(self.root_link, self.tip_link)

        # t_T_g = w.TransMatrix(self.goal_point)
        # r_T_tip_eval = w.TransMatrix(god_map.evaluate_expr(root_T_tip))

        # root_T_goal = r_T_tip_eval.dot(start_tip_T_current_tip).dot(t_T_g)

        root_T_goal = god_map.world.transform(self.root_link, self.goal_point)

        r_P_g = root_T_goal.to_position()
        r_P_c = root_T_tip.to_position()

        task = Task(name='VerticalMotion')
        self.add_task(task)

        task.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.velocity,
                                        weight=self.weight)


class Retracting(ObjectGoal):
    def __init__(self,
                 name: str = None,
                 distance: float = 0.3,
                 reference_frame: Optional[str] = None,
                 root_link: Optional[str] = None,
                 tip_link: Optional[str] = None,
                 velocity: float = 0.2,
                 threshold: float = 0.01,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Retract the tip link from the current position by the given distance.
        The exact direction is based on the given reference frame.

        :param name: Name of the goal, in this case Retracting.
        :param distance: Optional parameter to adjust the distance to move.
        :param reference_frame: Reference axis from which should be retracted. Is usually 'base_footprint' or 'hand_palm_link'
        :param root_link: Current root Link
        :param tip_link: Current tip link
        :param velocity: Desired velocity of this goal
        :param weight: weight of this goal

        """
        if name is None:
            name = 'Retracting'

        super().__init__(name=name)

        if reference_frame is None:
            reference_frame = 'base_footprint'
        if root_link is None:
            root_link = god_map.world.groups[god_map.world.robot_name].root_link.name
        if tip_link is None:
            tip_link = self.gripper_tool_frame
        self.distance = distance
        self.reference_frame = god_map.world.search_for_link_name(reference_frame)
        self.root_link = god_map.world.search_for_link_name(root_link)
        self.tip_link = god_map.world.search_for_link_name(tip_link)
        self.velocity = velocity
        self.weight = weight
        self.hand_frames = [self.gripper_tool_frame, 'hand_palm_link']

        tip_P_start = cas.TransMatrix()
        tip_P_start.reference_frame = self.tip_link
        reference_P_start = god_map.world.transform(self.reference_frame, tip_P_start)

        if self.reference_frame.short_name in self.hand_frames:
            reference_P_start.z -= self.distance
        else:
            reference_P_start.x -= self.distance

        self.goal_point = god_map.world.transform(self.tip_link, reference_P_start)
        # self.root_T_tip_start = god_map.world.compute_fk_np(self.root_link, self.tip_link)
        # self.start_tip_T_current_tip = np.eye(4)

        krg = KeepRotationGoal(tip_link='base_footprint',
                               weight=self.weight,
                               name='Retracting KRG')
        krg.start_condition = self.start_condition
        krg.pause_condition = self.pause_condition
        krg.end_condition = self.end_condition
        self.add_goal(krg)

        if 'base' not in self.tip_link.short_name:
            krg2 = KeepRotationGoal(tip_link=self.tip_link.short_name,
                                    weight=self.weight,
                                    name='Tip Link KRG')
            krg2.start_condition = self.start_condition
            krg2.pause_condition = self.pause_condition
            krg2.end_condition = self.end_condition
            self.add_goal(krg2)

        task = Task(name='Retracting')
        self.add_task(task)

        # start_tip_T_current_tip = w.TransMatrix(self.get_parameter_as_symbolic_expression('start_tip_T_current_tip'))
        root_T_tip = god_map.world.compose_fk_expression(self.root_link, self.tip_link)

        # t_T_g = w.TransMatrix(self.goal_point)
        # r_T_tip_eval = w.TransMatrix(god_map.evaluate_expr(root_T_tip))

        # root_T_goal = r_T_tip_eval.dot(start_tip_T_current_tip).dot(t_T_g)

        root_T_goal = god_map.world.transform(self.root_link, self.goal_point)

        r_P_g = root_T_goal.to_position()
        r_P_c = root_T_tip.to_position()

        task.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.velocity,
                                        weight=self.weight)

        distance = cas.euclidean_distance(r_P_g, r_P_c)
        self.observation_expression = cas.less_equal(distance, threshold)


class AlignHeight(ObjectGoal):
    goal_pose: cas.TransMatrix

    def __init__(self,
                 from_above: bool = False,
                 name: str = None,
                 object_name: Optional[str] = None,
                 goal_pose: Optional[cas.TransMatrix] = None,
                 object_height: float = 0.0,
                 root_link: Optional[str] = None,
                 tip_link: Optional[str] = None,
                 velocity: float = 0.2,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Align the tip link with the given goal_pose to prepare for further action (e.g. grasping or placing)

        :param from_above: whether action should be executed from above or not
        :param name: Name of the goal, in this case AlignHeight
        :param object_name: name of the object if added to world
        :param goal_pose: final destination pose
        :param object_height: height of the target object. Used as additional offset.
        :param root_link: Current root Link
        :param tip_link: Current tip link
        :param velocity: Desired velocity of this goal
        :param weight: weight of this goal
        """
        if name is None:
            name = 'AlignHeight'

        super().__init__(name=name)

        self.object_name = object_name

        # Get object from name
        if goal_pose is None:
            goal_pose, object_size = self.get_object_by_name(self.object_name)

            object_height = object_size.z

        try:
            god_map.world.search_for_link_name(goal_pose.reference_frame.short_name)
            self.goal_pose = goal_pose
        except:
            get_middleware().logwarn(f'Couldn\'t find {goal_pose.reference_frame}. Searching in tf.')
            self.goal_pose = god_map.world.compute_fk(god_map.world.search_for_link_name('map'), goal_pose)

        self.object_height = object_height

        if root_link is None:
            root_link = god_map.world.groups[god_map.world.robot_name].root_link.name
        if tip_link is None:
            tip_link = self.gripper_tool_frame

        self.root_link = god_map.world.search_for_link_name(root_link)
        self.tip_link = god_map.world.search_for_link_name(tip_link)

        self.velocity = velocity
        self.weight = weight

        self.from_above = from_above

        self.base_footprint = god_map.world.search_for_link_name('base_footprint')

        goal_point = self.goal_pose.to_position()

        base_to_tip = god_map.world.compute_fk(self.base_footprint, self.tip_link)

        offset = 0.02
        base_goal_point = god_map.world.transform(self.base_footprint, goal_point)
        base_goal_point.x = base_to_tip.x
        base_goal_point.z += (self.object_height / 2) + offset

        if self.from_above:
            # Tip facing downwards
            base_goal_point.z += 0.05

            base_V_g = cas.Vector3().from_xyz(0, 0, -1)
            base_V_g.reference_frame = self.base_footprint

            tip_V_g = cas.Vector3().from_xyz(self.gripper_forward.x, self.gripper_forward.y, self.gripper_forward.z)
            tip_V_g.reference_frame = self.tip_link

            base_V_x = cas.Vector3().from_xyz(x=1)
            base_V_x.reference_frame = self.base_footprint

            tip_V_x = cas.Vector3().from_xyz(x=1)
            tip_V_x.reference_frame = self.tip_link

            self.add_task(AlignPlanes(root_link=self.root_link,
                                      tip_link=self.tip_link,
                                      goal_normal=base_V_g,
                                      tip_normal=tip_V_g,
                                      name='APlane1'))

            self.add_task(AlignPlanes(root_link=self.root_link,
                                      tip_link=self.tip_link,
                                      goal_normal=base_V_x,
                                      tip_normal=tip_V_x,
                                      name='APlane2'))

        else:
            # Tip facing frontal
            krg_tip = KeepRotationGoal(tip_link=self.tip_link,
                                       weight=self.weight,
                                       name='KRG Tip facing frontal')
            krg_tip.start_condition = self.start_condition
            krg_tip.pause_condition = self.pause_condition
            krg_tip.end_condition = self.end_condition
            self.add_goal(krg_tip)

        krg_base = KeepRotationGoal(tip_link=self.base_footprint,
                                    weight=self.weight,
                                    name='KRG base_footprint')
        krg_base.start_condition = self.start_condition
        krg_base.pause_condition = self.pause_condition
        krg_base.end_condition = self.end_condition
        self.add_goal(krg_base)

        self.goal_point = god_map.world.transform(self.tip_link, base_goal_point)

        cp = CartesianPosition(root_link=self.root_link,
                               tip_link=self.tip_link,
                               goal_point=self.goal_point,
                               reference_velocity=self.velocity,
                               weight=self.weight,
                               name='AlignHeight CartesianPosition')
        cp.start_condition = self.start_condition
        cp.pause_condition = self.pause_condition
        cp.end_condition = self.end_condition
        self.add_task(cp)


class Placing(ObjectGoal):

    def __init__(self,
                 goal_pose: cas.TransMatrix,
                 align: str,
                 grasp: str,
                 name: str = None,
                 root_link: Optional[str] = None,
                 tip_link: Optional[str] = None,
                 velocity: float = 0.02,
                 weight: float = WEIGHT_ABOVE_CA):

        """
        Place an object. Use monitor_placing in python_interface.py in
         case of using the force-/torque-sensor to place objects.

        :param goal_pose: Goal pose for the object.
        :param align: States if the gripper should be rotated and in which "direction"
        :param name: Name of the goal, in this case Placing
        :param root_link: Current root Link
        :param tip_link: Current tip link
        :param velocity: Desired velocity of this goal
        :param weight: weight of this goal
        """
        if name is None:
            name = 'Placing'

        self.goal_pose = goal_pose
        self.velocity = velocity
        self.weight = weight
        self.align = align
        self.grasp = grasp

        # self.from_above = check_context_element('from_above', ContextFromAbove, context)

        super().__init__(name=name)

        if root_link is None:
            root_link = 'base_footprint'

        if tip_link is None:
            tip_link = self.gripper_tool_frame

        self.root_link = god_map.world.search_for_link_name(root_link)
        self.tip_link = god_map.world.search_for_link_name(tip_link)

        go = GraspObject(goal_pose=self.goal_pose,
                         align=self.align,
                         grasp=self.grasp,
                         root_link=self.root_link.short_name,
                         tip_link=self.tip_link.short_name,
                         velocity=self.velocity,
                         weight=self.weight)
        go.start_condition = self.start_condition
        go.pause_condition = self.pause_condition
        go.end_condition = self.pause_condition
        self.add_goal(go)

    # might need to be removed in the future, as soon as the old interface isn't in use anymore

    # def recovery_modifier(self) -> Dict:
    #     joint_states = {'arm_lift_joint': 0.03}
    #
    #     return joint_states


class Tilting(Goal):
    def __init__(self,
                 name: str = None,
                 direction: Optional[str] = None,
                 angle: Optional[float] = None,
                 tip_link: str = 'wrist_roll_joint'):
        """
        Tilts the given tip link into one direction by a given angle.

         :param name: Name of the goal, in this case Tilting
        :param direction: Direction in which to rotate the joint.
        :param angle: Amount how much the joint should be moved
        :param tip_link: The joint that should rotate. Default ensures correct usage for pouring.

        """
        if name is None:
            name = 'Tilting'
        super().__init__(name=name)

        max_angle = -2.0

        if angle is None:
            angle = max_angle

        if direction == 'right':
            angle = abs(angle)
        else:
            angle = abs(angle) * -1

        wrist_state = angle
        self.tip_link = tip_link

        self.goal_state = {self.tip_link: wrist_state}

        jpl = JointPositionList(goal_state=self.goal_state,
                                name='Tilting JointPositionList')
        jpl.start_condition = self.start_condition
        jpl.pause_condition = self.pause_condition
        jpl.end_condition = self.end_condition
        self.add_task(jpl)


class TakePose(Goal):
    def __init__(self,
                 pose_keyword: str,
                 name: str = None):
        """
        Get into a predefined pose with a given keyword.
        Used to get into complete poses. To move only specific joints use 'JointPositionList'

        :param pose_keyword: Keyword for the given poses
        """
        if name is None:
            name = f'TakePose-{pose_keyword}'
        super().__init__(name=name)

        if pose_keyword == TakePoseTypes.PARK.value:
            head_pan_joint = 0.0
            head_tilt_joint = 0.0
            arm_lift_joint = 0.0
            arm_flex_joint = 0.0
            arm_roll_joint = -1.5
            wrist_flex_joint = -1.5
            wrist_roll_joint = 0.0

        elif pose_keyword == TakePoseTypes.PARK_LEFT.value:
            head_pan_joint = 0.0
            head_tilt_joint = 0.0
            arm_lift_joint = 0.0
            arm_flex_joint = 0.0
            arm_roll_joint = 1.5
            wrist_flex_joint = -1.5
            wrist_roll_joint = 0.0

        elif pose_keyword == TakePoseTypes.PERCEIVE.value:
            head_pan_joint = 0.0
            head_tilt_joint = -0.65
            arm_lift_joint = 0.25
            arm_flex_joint = 0.0
            arm_roll_joint = 1.5
            wrist_flex_joint = -1.5
            wrist_roll_joint = 0.0

        elif pose_keyword == TakePoseTypes.ASSISTANCE.value:
            head_pan_joint = 0.0
            head_tilt_joint = 0.0
            arm_lift_joint = 0.0
            arm_flex_joint = 0.0
            arm_roll_joint = -1.5
            wrist_flex_joint = -1.5
            wrist_roll_joint = 1.6

        elif pose_keyword == TakePoseTypes.PRE_ALIGN_HEIGHT.value:
            head_pan_joint = 0.0
            head_tilt_joint = 0.0
            arm_lift_joint = 0.0
            arm_flex_joint = 0.0
            arm_roll_joint = 0.0
            wrist_flex_joint = -1.5
            wrist_roll_joint = 0.0

        elif pose_keyword == TakePoseTypes.CARRY.value:
            head_pan_joint = 0.0
            head_tilt_joint = -0.65
            arm_lift_joint = 0.0
            arm_flex_joint = -0.43
            arm_roll_joint = 0.0
            wrist_flex_joint = -1.17
            wrist_roll_joint = -1.62

        elif pose_keyword == TakePoseTypes.TEST.value:
            head_pan_joint = 0.0
            head_tilt_joint = 0.0
            arm_lift_joint = 0.38
            arm_flex_joint = -1.44
            arm_roll_joint = 0.0
            wrist_flex_joint = -0.19
            wrist_roll_joint = 0.0

        elif pose_keyword == TakePoseTypes.PRE_TRAY.value:
            head_pan_joint = 0.0
            head_tilt_joint = 0.0
            arm_lift_joint = 0.6
            arm_flex_joint = -1.44
            arm_roll_joint = 0.0
            wrist_flex_joint = -1.22
            wrist_roll_joint = np.pi / 2

        else:
            get_middleware().loginfo(f'{pose_keyword} is not a valid pose')
            return

        joint_states = {
            'head_pan_joint': head_pan_joint,
            'head_tilt_joint': head_tilt_joint,
            'arm_lift_joint': arm_lift_joint,
            'arm_flex_joint': arm_flex_joint,
            'arm_roll_joint': arm_roll_joint,
            'wrist_flex_joint': wrist_flex_joint,
            'wrist_roll_joint': wrist_roll_joint}
        self.goal_state = joint_states

        jpl = JointPositionList(goal_state=self.goal_state, name='TakePoseJPL')
        jpl.start_condition = self.start_condition
        jpl.pause_condition = self.pause_condition
        jpl.end_condition = self.end_condition
        self.add_task(jpl)


class Mixing(Goal):
    def __init__(self,
                 name=None,
                 mixing_time: float = 20,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Simple Mixing motion.

        :param mixing_time: States how long this goal should be executed.
        :param weight: weight of this goal
        """
        if name is None:
            name = 'Mixing'
        super().__init__(name=name)

        self.weight = weight

        target_speed = 1

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='wrist_roll_joint',
                                                                 joint_center=0.0,
                                                                 joint_range=0.9,
                                                                 trajectory_length=mixing_time,
                                                                 target_speed=target_speed))

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='wrist_flex_joint',
                                                                 joint_center=-1.3,
                                                                 joint_range=0.2,
                                                                 trajectory_length=mixing_time,
                                                                 target_speed=target_speed))

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='arm_roll_joint',
                                                                 joint_center=0.0,
                                                                 joint_range=0.1,
                                                                 trajectory_length=mixing_time,
                                                                 target_speed=target_speed))


class JointRotationGoalContinuous(Goal):
    def __init__(self,
                 joint_name: str,
                 joint_center: float,
                 joint_range: float,
                 name: str = None,
                 trajectory_length: float = 20,
                 target_speed: float = 1,
                 period_length: float = 1.0):
        """
        Rotate a joint continuously around a center. The execution time and speed is variable.

        :param joint_name: joint name that should be rotated
        :param joint_center: Center of the rotation point
        :param joint_range: Range of the rotational movement. Note that this is calculated + and - joint_center.
        :param trajectory_length: length of this goal in seconds.
        :param target_speed: execution speed of this goal. Adjust when the trajectory is not executed right
        :param period_length: length of the period that should be executed. Adjust when the trajectory is not executed right.
        """
        if name is None:
            name = 'JointRotationGoalContinuous'
        super().__init__(name=name)
        self.joint = god_map.world.search_for_joint_name(joint_name)
        self.target_speed = target_speed
        self.trajectory_length = trajectory_length
        self.joint_center = joint_center
        self.joint_range = joint_range
        self.period_length = period_length

    def make_constraints(self):
        time = self.traj_time_in_seconds()
        joint_position = self.get_joint_position_symbol(self.joint)

        joint_goal = self.joint_center + (w.cos(time * np.pi * self.period_length) * self.joint_range)

        god_map.debug_expression_manager.add_debug_expression(f'{self.joint.short_name}_goal', joint_goal)
        god_map.debug_expression_manager.add_debug_expression(f'{self.joint.short_name}_position', joint_position)

        self.add_position_constraint(expr_current=joint_position,
                                     expr_goal=joint_goal,
                                     reference_velocity=self.target_speed,
                                     weight=w.if_greater(time, self.trajectory_length, 0, WEIGHT_ABOVE_CA),
                                     name=self.joint.short_name)


class KeepRotationGoal(Goal):
    def __init__(self,
                 tip_link: str,
                 name: str = None,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Use this if a specific link should not rotate during a goal execution. Typically used for the hand.

        :param tip_link: link that shall keep its rotation
        :param weight: weight of this goal
        """
        if name is None:
            name = 'KeepRotationGoal'

        super().__init__(name=name)

        self.tip_link = god_map.world.search_for_link_name(tip_link)
        self.weight = weight

        tip_orientation = cas.RotationMatrix()
        tip_orientation.reference_frame = self.tip_link

        co = CartesianOrientation(root_link=PrefixName('map'),
                                  tip_link=self.tip_link,
                                  goal_orientation=tip_orientation,
                                  weight=self.weight,
                                  name=f'CartesianOrientation of {self.name}')
        self.add_task(co)


class GraspWithForceTorqueGoal(Goal):
    def __init__(self,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 handle_name: PrefixName,
                 tip_grasp_axis: cas.Vector3,
                 bar_axis: cas.Vector3,
                 tip_retract: cas.Point3,
                 handle_align_axis: cas.Vector3,
                 tip_align_axis: cas.Vector3,
                 grasp_axis_offset: cas.Vector3,
                 pre_grasp_axis_offset: cas.Vector3,
                 hinge_joint: PrefixName,
                 bar_length: float = 0.01,
                 timeout: float = 10,
                 ft_topic: str = '/filtered_raw/diff',
                 ft_grasp_ref_speed: float = 1,
                 camera_link: Optional[PrefixName] = None,
                 tip_push: Optional[cas.Point3] = None,
                 handle_correction_offset: Optional[cas.Point3] = None,
                 name: str = None):
        """
        Complex grasping motion using the ForceTorqueMonitor.

        :param root_link: root-link of the kinematic chain.
        :param tip_link: tip-link of the kinematic chain.
        :param handle_name: LinkName of the environment-link that is to be grasped.
        :param tip_grasp_axis: axis of tip-link that is to be aligned with bar axis.
        :param bar_axis: axis of the handle along which the handlebar is.
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
        :param name: Name of the Goal
        """
        if name is None:
            name = 'GraspWithForceTorqueGoal'
        super().__init__(name=name)

        pre_grasp_name = 'pre grasp'

        bar_center = cas.Point3()
        bar_center.reference_frame = handle_name

        self.reference_linear_velocity = 0.1 * ft_grasp_ref_speed
        self.reference_angular_velocity = 0.5 * ft_grasp_ref_speed

        door_hinge_lock = {hinge_joint.short_name: 0}
        jpl_hinge_lock = JointPositionList(goal_state=door_hinge_lock,
                                           name='Lock Hinge while grasp')
        jpl_hinge_lock.start_condition = self.start_condition
        self.add_task(jpl_hinge_lock)

        door_joint = PrefixName(name='iai_kitchen:arena:door_hinge_joint', prefix='iai_kitchen')
        init_vector = cas.Vector3(reference_frame=camera_link)

        if camera_link is not None and tip_push is not None:
            handle_correction = HandleOffsetCorrection(root_link=root_link,
                                                       tip_link=camera_link,
                                                       goal_vector=init_vector,
                                                       door_move_joint=door_joint,
                                                       threshold=30,
                                                       magic=500)
            handle_correction.start_condition = pre_grasp_name
            handle_correction.end_condition = handle_correction
            self.add_monitor(handle_correction)

            end_condition_pre_grasp = handle_correction

            if handle_correction_offset is not None:
                midpoint_offset = CartesianPosition(root_link=root_link,
                                                    tip_link=tip_link,
                                                    goal_point=handle_correction_offset,
                                                    name='offset handle midpoint',
                                                    threshold=0.001)
                midpoint_offset.start_condition = handle_correction
                midpoint_offset.end_condition = midpoint_offset
                self.add_task(midpoint_offset)

                next_condition = midpoint_offset
            else:
                next_condition = handle_correction
        else:
            next_condition = pre_grasp_name
            end_condition_pre_grasp = pre_grasp_name

        pre_grasp = GraspBarOffset(name=pre_grasp_name,
                                   root_link=root_link,
                                   tip_link=tip_link,
                                   tip_grasp_axis=tip_grasp_axis,
                                   bar_center=bar_center,
                                   bar_axis=bar_axis,
                                   bar_length=bar_length,
                                   grasp_axis_offset=grasp_axis_offset,
                                   handle_link=handle_name)
        pre_grasp.start_condition = self.start_condition
        pre_grasp.end_condition = end_condition_pre_grasp
        self.add_goal(pre_grasp)

        ap_pre_grasp = AlignPlanes(name='grasp align',
                                   root_link=root_link,
                                   tip_link=tip_link,
                                   tip_normal=tip_align_axis,
                                   goal_normal=handle_align_axis)
        ap_pre_grasp.start_condition = self.start_condition
        self.add_task(ap_pre_grasp)

        sleep_cancel = Sleep(seconds=timeout, name='ft sleep cancel')
        sleep_cancel.start_condition = next_condition
        self.add_monitor(sleep_cancel)

        ft_monitor = PayloadForceTorque(threshold_enum=ForceTorqueThresholds.DOOR.value,
                                        object_type='',
                                        name='grasp ft monitor',
                                        topic=ft_topic)
        ft_monitor.start_condition = next_condition
        self.add_monitor(ft_monitor)

        if camera_link is not None and tip_push is not None:
            ft_grasp = CartesianPosition(root_link=root_link,
                                         tip_link=tip_link,
                                         goal_point=tip_push,
                                         name='ft grasp',
                                         reference_velocity=self.reference_linear_velocity,
                                         threshold=0.001)
            ft_grasp.start_condition = next_condition
            ft_grasp.end_condition = ft_monitor
            self.add_task(ft_grasp)
        else:
            ft_grasp = GraspBarOffset(name='ft grasp',
                                      root_link=root_link,
                                      tip_link=tip_link,
                                      tip_grasp_axis=tip_grasp_axis,
                                      bar_center=bar_center,
                                      bar_axis=bar_axis,
                                      bar_length=bar_length,
                                      grasp_axis_offset=pre_grasp_axis_offset,
                                      reference_linear_velocity=self.reference_linear_velocity,
                                      reference_angular_velocity=self.reference_angular_velocity,
                                      handle_link=handle_name)
            ft_grasp.start_condition = next_condition
            ft_grasp.end_condition = ft_monitor
            self.add_goal(ft_grasp)

        ap_tip_grasp = AlignPlanes(name='tip grasp align',
                                   root_link=root_link,
                                   tip_link=tip_link,
                                   tip_normal=tip_grasp_axis,
                                   goal_normal=bar_axis)
        ap_tip_grasp.start_condition = next_condition
        self.add_task(ap_tip_grasp)

        retract = CartesianPosition(root_link=root_link,
                                    tip_link=tip_link,
                                    goal_point=tip_retract,
                                    name='retract after ft',
                                    reference_velocity=self.reference_linear_velocity,
                                    threshold=0.001)
        retract.start_condition = ft_monitor
        retract.end_condition = retract
        self.add_task(retract)

        ft_cancel = CancelMotion(exception=ObjectForceTorqueThresholdException('Door not touched!'),
                                 name='FT CancelMotion')
        ft_cancel.start_condition = f'not {ft_monitor} and {sleep_cancel}'
        self.add_monitor(ft_cancel)

        self.observation_expression = retract.observation_expression


class OpenDoorGoal(Goal):
    def __init__(self,
                 tip_link: PrefixName,
                 handle_name: PrefixName,
                 handle_limit: float,
                 hinge_limit: float,
                 root_link: PrefixName = None,
                 tip_normal: cas.Vector3 = None,
                 goal_normal: cas.Vector3 = None,
                 name: str = None):
        """
        Use this, if you have grasped a door handle and want to open the door and handle

        :param tip_link: end effector that is grasping the handle
        :param handle_name: link that is grasped by the tip_link
        :param name: name of the goal
        :param handle_limit: Limit for how far the handle will be opened
        :param hinge_limit: Limit for how far the door-hinge will be opened
        :param root_link: Root-link for alignment of gripper to handle
        :param tip_normal: Gripper axis that is to be aligned with goal_normal
        :param goal_normal: Handle axis that is to be aligned with tip_normal
        """
        if name is None:
            name = 'OpenDoorGoal'
        if tip_normal is None:
            tip_normal = cas.Vector3()
            tip_normal.reference_frame = god_map.world.search_for_link_name('base_link', 'hsrb')
            tip_normal.x = 1
        if goal_normal is None:
            goal_normal = cas.Vector3()
            goal_normal.reference_frame = handle_name
            goal_normal.z = -1
        if root_link is None:
            root_link = god_map.world.search_for_link_name('map')
        super().__init__(name=name)

        handle_name = handle_name
        handle_frame_id = god_map.world.get_movable_parent_joint(handle_name)
        link_id = god_map.world.get_parent_link_of_joint(handle_frame_id)
        door_hinge_id = god_map.world.get_movable_parent_joint(link_id)

        _, max_limit_handle = god_map.world.compute_joint_limits(handle_frame_id, 0)
        min_limit_hinge, max_limit_hinge = god_map.world.compute_joint_limits(door_hinge_id, 0)

        if handle_limit is None:
            limit_handle = max_limit_handle
        else:
            limit_handle = min(max_limit_handle, handle_limit)

        if hinge_limit is None:
            limit_hinge = min_limit_hinge
        else:
            limit_hinge = max(min_limit_hinge, hinge_limit)

        handle_state = {handle_frame_id: limit_handle}
        handle_state_monitor = JointGoalReached(goal_state=handle_state,
                                                threshold=0.005,
                                                name=f'{name}_handle_joint_monitor')
        self.add_monitor(handle_state_monitor)

        open_goal = Open(tip_link=tip_link,
                         environment_link=handle_name,
                         goal_joint_state=limit_handle,
                         name='OpenHandle',
                         max_velocity=0.3)
        self.add_goal(open_goal)

        jpl = JointPositionList(goal_state={door_hinge_id: max_limit_hinge},
                                weight=WEIGHT_ABOVE_CA,
                                name='DoorHinge')
        jpl.end_condition = handle_state_monitor
        self.add_task(jpl)

        apl = AlignPlanes(root_link=root_link,
                          tip_link=tip_normal.reference_frame,
                          goal_normal=goal_normal,
                          tip_normal=tip_normal,
                          name='AlignBaseWithDoor')
        apl.start_condition = handle_state_monitor
        self.add_task(apl)

        open_goal2 = Open(tip_link=tip_link,
                          environment_link=link_id,
                          goal_joint_state=limit_hinge,
                          name='OpenHinge',
                          max_velocity=0.5)
        open_goal2.start_condition = handle_state_monitor
        self.add_goal(open_goal2)

        goal_state = {door_hinge_id: limit_hinge}
        hinge_state_monitor = JointGoalReached(name='HingeMonitor', goal_state=goal_state)
        hinge_state_monitor.start_condition = handle_state_monitor
        self.add_monitor(hinge_state_monitor)

        self.observation_expression = hinge_state_monitor.observation_expression


class MoveAroundHinge(Goal):
    old_position_monitor: Monitor = None
    tip_gripper_axis: cas.Vector3 = None
    root_V_tip_grasp_axis: cas.Vector3 = None
    root_V_object_rotation_axis: cas.Vector3 = None

    def __init__(self,
                 handle_name: PrefixName,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 tip_gripper_axis: cas.Vector3 = None,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 weight: float = WEIGHT_BELOW_CA,
                 goal_angle: float = None,
                 multipliers: Optional[List[Tuple[float, float, str]]] = None,
                 offset: Optional[cas.Vector3] = None,
                 align_gripper: MoveAroundHingeAlign = MoveAroundHingeAlign.LAST,
                 name: str = None):
        """
        Adds Points to move around the hinge to a given handle

        :param handle_name: full frame id of the handle
        :param root_link: root link of the kinematic chain
        :param tip_link: tip link of the kinematic chain
        :param reference_linear_velocity: m/s
        :param weight:
        :param name: Name of the goal
        """
        if name is None:
            name = 'MoveAroundHingeGoal'
        super().__init__(name=name)
        if multipliers is None:
            multipliers = [(11 / 10, -0.7, 'down_short'),
                           (7 / 5, -0.3, 'down_long'),
                           (7 / 5, 0.4, 'up_long')]
        if offset is None:
            offset = cas.Vector3().from_xyz(0, 0, 0, root_link)

        self.weight = weight
        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity

        self.handle_frame_id = self.tip_link = god_map.world.search_for_link_name(handle_name)

        hinge_joint = god_map.world.get_movable_parent_joint(self.handle_frame_id)
        door_hinge_frame_id = god_map.world.get_parent_link_of_link(self.handle_frame_id)

        self.tip_link = tip_link
        self.root_link = root_link

        root_T_tip = god_map.world.compose_fk_expression(self.root_link, self.tip_link)
        root_P_tip = root_T_tip.to_position()
        object_joint_angle = god_map.world.state[hinge_joint].position

        object_V_object_rotation_axis = cas.Vector3(god_map.world.get_joint(hinge_joint).axis)
        root_T_door_expr = god_map.world.compose_fk_expression(self.root_link, door_hinge_frame_id)
        root_V_offset = god_map.world.transform(self.root_link, offset)
        root_T_offset = cas.TransMatrix().from_xyz_rpy(root_V_offset.x,
                                                       root_V_offset.y,
                                                       root_V_offset.z,
                                                       reference_frame=offset.reference_frame)
        root_T_door_expr = root_T_offset.dot(root_T_door_expr)

        if tip_gripper_axis is not None:
            tip_gripper_axis.scale(1)
            self.tip_gripper_axis = tip_gripper_axis

            tip_V_tip_grasp_axis = cas.Vector3(self.tip_gripper_axis)
            self.root_V_tip_grasp_axis = cas.dot(root_T_tip, tip_V_tip_grasp_axis)
            self.root_V_object_rotation_axis = cas.dot(root_T_door_expr, object_V_object_rotation_axis)

        door_P_handle = god_map.world.compute_fk(door_hinge_frame_id, self.handle_frame_id).to_position()
        temp_point = door_P_handle.to_np()
        # axis pointing in the direction of handle frame from door joint frame
        direction_axis = np.argmax(abs(temp_point[0:3]))

        root_P_top_chain = []

        for i, (axis_multi, angle_multi, goal_name) in enumerate(multipliers):
            door_P_intermediate_point = np.zeros(3)
            door_P_intermediate_point[direction_axis] = temp_point[direction_axis] * axis_multi
            door_P_intermediate_point = cas.Point3([door_P_intermediate_point[0],
                                                    door_P_intermediate_point[1],
                                                    door_P_intermediate_point[2]])

            # # point w.r.t door
            if goal_angle is None:
                desired_angle = object_joint_angle * angle_multi  # just chose 1/2 of the goal angle
            else:
                desired_angle = goal_angle * angle_multi

            # find point w.r.t rotated door in local frame
            door_R_door_rotated = cas.RotationMatrix.from_axis_angle(axis=object_V_object_rotation_axis,
                                                                     angle=desired_angle)
            door_T_door_rotated = cas.TransMatrix(door_R_door_rotated)
            # as the root_T_door is already pointing to a completely rotated door, we invert desired angle to get to the
            # intermediate point
            door_rotated_P_top = cas.dot(door_T_door_rotated.inverse(), door_P_intermediate_point)
            root_P_top = cas.dot(cas.TransMatrix(root_T_door_expr), door_rotated_P_top)

            root_P_top_chain.append((root_P_top, goal_name))

        for i, (root_P_top, goal_name) in enumerate(root_P_top_chain):
            god_map.debug_expression_manager.add_debug_expression(f'goal_point_{goal_name}', root_P_top,
                                                                  color=ColorRGBA(0, 0.5, 0.5, 1))

            task = Task(name=goal_name)
            self.add_task(task)

            if self.old_position_monitor is None:
                position_monitor = Monitor(name=f'{goal_name}_pos_monitor')
            else:
                position_monitor = Monitor(name=f'{goal_name}_pos_monitor')
                position_monitor.start_condition = self.old_position_monitor

            distance_to_point = cas.euclidean_distance(root_P_tip, root_P_top)
            point_reached = cas.less(distance_to_point, 0.01)
            position_monitor.observation_expression = point_reached
            self.add_monitor(position_monitor)

            task.add_point_goal_constraints(frame_P_current=root_T_tip.to_position(),
                                            frame_P_goal=root_P_top,
                                            reference_velocity=self.reference_linear_velocity,
                                            weight=self.weight)

            # Add Vector-Align for better alignment to push later
            if (((i == len(root_P_top_chain) - 1 and align_gripper == MoveAroundHingeAlign.LAST)
                 or (align_gripper == MoveAroundHingeAlign.ALL))
                    and self.tip_gripper_axis is not None):
                task.add_vector_goal_constraints(frame_V_current=self.root_V_tip_grasp_axis,
                                                 frame_V_goal=self.root_V_object_rotation_axis,
                                                 reference_velocity=self.reference_angular_velocity,
                                                 weight=self.weight)

            if i == 0:
                task.start_condition = self.start_condition
                task.end_condition = position_monitor.name
            elif i == len(root_P_top_chain) - 1:
                end_con = f'({self.end_condition}) or {position_monitor.name}'
                task.start_condition = self.old_position_monitor.name
                task.end_condition = end_con
            else:
                task.start_condition = self.old_position_monitor.name
                task.end_condition = position_monitor.name

            self.old_position_monitor = position_monitor

        self.observation_expression = self.old_position_monitor.observation_expression


class GraspBarOffset(Goal):
    bar_axis: cas.Vector3
    tip_grasp_axis: cas.Vector3
    bar_center: cas.Point3
    grasp_axis_offset: cas.Vector3

    def __init__(self,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 tip_grasp_axis: cas.Vector3,
                 bar_center: cas.Point3,
                 bar_axis: cas.Vector3,
                 bar_length: float,
                 grasp_axis_offset: cas.Vector3,
                 handle_link: PrefixName,
                 threshold: float = 0.01,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 weight: float = WEIGHT_ABOVE_CA,
                 name: Optional[str] = None):
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
        """
        self.root = root_link
        self.tip = tip_link
        if name is None:
            name = f'{self.__class__.__name__}/{self.root}/{self.tip}'
        super().__init__(name=name)

        handle_P_bar_center = god_map.world.transform(handle_link, bar_center)
        root_P_bar_center = god_map.world.compose_fk_evaluated_expression(self.root, handle_link).dot(handle_P_bar_center)
        grasp_axis_offset = god_map.world.transform(self.root, grasp_axis_offset)

        tip_grasp_axis = god_map.world.transform(self.tip, tip_grasp_axis)
        tip_grasp_axis = tip_grasp_axis / tip_grasp_axis.norm()

        bar_axis = god_map.world.transform(self.root, bar_axis)
        bar_axis = bar_axis / bar_axis.norm()

        self.bar_axis = bar_axis
        self.tip_grasp_axis = tip_grasp_axis
        self.bar_center = bar_center
        self.bar_length = bar_length
        self.grasp_axis_offset = grasp_axis_offset
        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity
        self.weight = weight

        root_V_bar_axis = self.bar_axis
        tip_V_tip_grasp_axis = self.tip_grasp_axis
        root_V_bar_offset = self.grasp_axis_offset

        root_T_tip = god_map.world.compose_fk_expression(self.root, self.tip)

        root_V_tip_normal = cas.dot(root_T_tip, tip_V_tip_grasp_axis)

        task = Task(name=f'{name}_task')
        self.add_task(task)

        task.add_vector_goal_constraints(frame_V_current=root_V_tip_normal,
                                         frame_V_goal=root_V_bar_axis,
                                         reference_velocity=self.reference_angular_velocity,
                                         weight=self.weight)

        root_P_tip = god_map.world.compose_fk_expression(self.root, self.tip).to_position()

        root_P_bar_center = root_P_bar_center + root_V_bar_offset

        root_P_line_start = root_P_bar_center + root_V_bar_axis * self.bar_length / 2
        root_P_line_end = root_P_bar_center - root_V_bar_axis * self.bar_length / 2

        dist, nearest = cas.distance_point_to_line_segment(root_P_tip, root_P_line_start, root_P_line_end)

        task.add_point_goal_constraints(frame_P_current=root_T_tip.to_position(),
                                        frame_P_goal=nearest,
                                        reference_velocity=self.reference_linear_velocity,
                                        weight=self.weight)
        # self.connect_monitors_to_all_tasks(start_condition, pause_condition, end_condition)

        god_map.debug_expression_manager.add_debug_expression(f'{name}_nearest', nearest)
        god_map.debug_expression_manager.add_debug_expression('tip V tip grasp axis', tip_V_tip_grasp_axis)

        self.observation_expression = cas.less_equal(dist, threshold)


def check_context_element(name: str,
                          context_type,
                          context):
    if name in context:
        if isinstance(context[name], context_type):
            return context[name].content
        else:
            return context[name]


def multiply_vector(vec: cas.Vector3,
                    number: int):
    return cas.Vector3.from_xyz(vec.x * number, vec.y * number, vec.z * number)
