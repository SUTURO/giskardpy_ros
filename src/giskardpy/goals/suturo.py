from copy import deepcopy
from typing import Optional, List, Dict

import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped, Quaternion

from giskardpy import casadi_wrapper as w, identifier
from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPosition, CartesianOrientation
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA, ForceSensorGoal
from giskardpy.goals.joint_goals import JointPosition, JointPositionList
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.qp.constraint import EqualityConstraint
from giskardpy.utils.logging import loginfo, logwarn
from giskardpy.utils.math import inverse_frame


class ObjectGoal(Goal):
    def __init__(self):
        """
        Inherit from this class if the goal tries to get the object by name form the tf frames
        """

        super().__init__()

    def get_object_by_name(self, object_name):
        try:
            loginfo('trying to get objects with name')

            object_link = self.world.get_link(object_name)
            object_geometry: LinkGeometry = object_link.collisions[0]

            goal_pose = self.world.compute_fk_pose('map', object_name)

            loginfo(f'goal_pose by name: {goal_pose}')

            # Declare instance of geometry
            if isinstance(object_geometry, BoxGeometry):
                object_type = 'box'
                object_geometry: BoxGeometry = object_geometry
                object_size = Vector3(object_geometry.width, object_geometry.depth, object_geometry.height)

            elif isinstance(object_geometry, CylinderGeometry):
                object_type = 'cylinder'
                object_geometry: CylinderGeometry = object_geometry
                object_size = Vector3(object_geometry.radius, object_geometry.radius, object_geometry.height)

            elif isinstance(object_geometry, SphereGeometry):
                object_type = 'sphere'
                object_geometry: SphereGeometry = object_geometry
                object_size = Vector3(object_geometry.radius, object_geometry.radius, object_geometry.radius)

            else:
                raise Exception('Not supported geometry')

            loginfo(f'Got geometry: {object_type}')
            return goal_pose, object_size

        except:
            loginfo('Could not get geometry from name')
            return None

    def convert_list_to_size(self, size: List) -> Vector3:
        if len(size) == 3:
            # Type Box
            return Vector3(size[0], size[1], size[2])

        if len(size) == 2:
            # Type Cylinder
            return Vector3(size[0], size[0], size[1])

        if len(size) == 1:
            # Type Sphere
            return Vector3(size[0], size[0], size[0])

    def try_to_get_link(self, expected: str):
        try:
            link = self.world.search_for_link_name(expected)
            return link
        except:
            logwarn(f'Could not find {expected}.')
            raise Exception  # TODO:  CouldFindLinkException

    def try_to_get_size_from_geometry(self,
                                      name: str,
                                      geometry: LinkGeometry,
                                      frame_fallback: str,
                                      size_fallback: Vector3):

        if isinstance(geometry, BoxGeometry):
            object_size = Vector3(x=geometry.width, y=geometry.depth, z=geometry.height)

            reference_frame = name

        else:
            object_size = Vector3(x=size_fallback.x, y=size_fallback.y, z=size_fallback.z)

            reference_frame = frame_fallback

        return reference_frame, object_size


class TestForceSensorGoal(ForceSensorGoal):
    def __init__(self,
                 goal_pose: PoseStamped = None,
                 **kwargs):
        super().__init__()

        self.add_constraints_of_goal(Retracting(object_name=''))

    def make_constraints(self):
        pass

    def __str__(self):
        return super().__str__()

    def goal_cancel_condition(self) -> [(str, str, w.Expression)]:
        x_force_threshold = w.Expression(0.0)
        x_force_condition = ['x_force', '<=', x_force_threshold]

        expression = [x_force_condition]

        return expression

    def recovery(self) -> Dict:
        joint_states = {'arm_lift_joint': 0.01}

        return joint_states


class SequenceGoal(Goal):
    def __init__(self,
                 motion_sequence: Dict,
                 goal_type_seq=None,
                 kwargs_seq=None,
                 **kwargs):
        super().__init__()

        self.goal_type_seq = goal_type_seq
        self.kwargs_seq = kwargs_seq
        self.motion_sequence = motion_sequence

        self.current_goal_number = 0
        self.eq_weights = []
        self.goal_summary = []

        for index, (current_goal, goal_args) in enumerate(zip(self.goal_type_seq, self.kwargs_seq)):
            params = deepcopy(goal_args)
            params['suffix'] = index

            goal_instance: Goal = current_goal(**params)
            self.add_constraints_of_goal(goal_instance)

            self.goal_summary.append(goal_instance)

    def make_constraints(self):

        constraints: Dict[str, EqualityConstraint] = self._equality_constraints

        eq_constraint_suffix = [f'_suffix:{x}' for x in range(len(self.goal_type_seq))]

        values = constraints.items()
        for goal_number, suffix_text in enumerate(eq_constraint_suffix):

            ordered_eq_constraints = [x[1] for x in values if suffix_text in x[0]]

            eq_constraint_weights = [1] * len(ordered_eq_constraints)
            self.eq_weights.append(eq_constraint_weights)

            for eq_number, constraint in enumerate(ordered_eq_constraints):
                compiled = constraint.capped_error(self.sample_period).compile()
                s = self.god_map.to_symbol(self._get_identifier() + ['asdf', (goal_number, eq_number, compiled)])

                expr = w.Expression(s)
                constraint.quadratic_weight = constraint.quadratic_weight * expr

    def asdf(self, goal_number, eq_number, compiled_constraint):

        if goal_number != self.current_goal_number:
            return 0

        eq_constraint_error = compiled_constraint.fast_call(self.god_map.get_values(compiled_constraint.str_params))

        if abs(eq_constraint_error) < 0.001:
            self.eq_weights[goal_number][eq_number] = 0
        else:
            self.eq_weights[goal_number][eq_number] = 1

        goal_not_finished = any(self.eq_weights[goal_number])
        if goal_not_finished:
            return 1

        self.current_goal_number += 1

        if self.current_goal_number >= len(self.goal_summary):
            return 0

        self.goal_summary[self.current_goal_number].update_params()

        return 0

    def __str__(self) -> str:
        return super().__str__()


class MoveGripper(Goal):
    def __init__(self,
                 gripper_state: str,
                 suffix=''):
        """
        Open / CLose Gripper.
        Current implementation is not final and will be replaced with a follow joint trajectory connection.

        :param gripper_state: True to open gripper; False to close gripper.
        """

        super().__init__()

        self.suffix = suffix

        self.gripper_state = gripper_state

        if self.gripper_state == 'open':
            self.gripper_function = self.god_map.get_data(identifier=identifier.gripper_controller)
            self.gripper_function(0.8)

        elif self.gripper_state == 'neutral':

            self.gripper_function = self.god_map.get_data(identifier=identifier.gripper_trajectory)
            self.gripper_function(0.5)

        elif self.gripper_state == 'close':
            self.gripper_function = self.god_map.get_data(identifier=identifier.gripper_controller)
            self.gripper_function(-0.8)

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class Reaching(ObjectGoal):
    def __init__(self,
                 context,
                 object_name: Optional[str] = '',
                 object_shape: Optional[str] = '',
                 goal_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Concludes Reaching type goals.
        Executes them depending on the given context action.
        Examples: grasping, placing or pouring
        """
        super().__init__()
        self.context = context
        self.object_name = object_name
        self.object_shape = object_shape
        self.root_str = root_link
        self.tip_str = tip_link
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.action = context['action']
        self.from_above = False
        self.vertical_align = False

        if 'from_above' in context:
            self.from_above = context['from_above']

        if 'vertical_align' in context:
            self.vertical_align = context['vertical_align']

        # Get object geometry from name
        if goal_pose is None:
            self.goal_pose, self.object_size = self.get_object_by_name(self.object_name)

            self.reference_frame = self.object_name

        else:
            self.goal_pose = goal_pose
            self.object_size = object_size  # self.convert_list_to_size(object_size)

            self.reference_frame = 'base_link'

            logwarn(f'Deprecated warning: Please add object to giskard and set object name.')

        if self.action == 'grasping':
            if self.object_shape == 'sphere' or self.object_shape == 'cylinder':
                radius = self.object_size.x
            else:

                object_in_world = self.get_object_by_name(self.object_name) is not None

                if object_in_world:
                    radius = -0.04  # shelf
                    radius = 0.02  # drawer

                else:
                    radius = max(min(0.08, self.object_size.x / 2), 0.05)

            self.add_constraints_of_goal(GraspObject(goal_pose=self.goal_pose,
                                                     object_size=self.object_size,
                                                     reference_frame_alignment=self.reference_frame,
                                                     frontal_offset=radius,
                                                     from_above=self.from_above,
                                                     vertical_align=self.vertical_align,
                                                     root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     velocity=self.velocity,
                                                     weight=self.weight,
                                                     suffix=self.suffix))

        elif self.action == 'placing':
            # Todo: Place from above: use radius for object height offset
            if self.object_shape == 'sphere' or self.object_shape == 'cylinder':
                radius = self.object_size.x
            else:
                radius = 0.0

            # Placing positions are calculated in planning in clean the table.
            # Apply height offset only when placing frontal
            if not self.from_above:
                self.goal_pose.pose.position.z += (self.object_size.z / 2) + 0.02

            self.add_constraints_of_goal(GraspObject(goal_pose=self.goal_pose,
                                                     object_size=self.object_size,
                                                     reference_frame_alignment=self.reference_frame,
                                                     frontal_offset=radius,
                                                     from_above=self.from_above,
                                                     vertical_align=self.vertical_align,
                                                     root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     velocity=self.velocity,
                                                     weight=self.weight,
                                                     suffix=self.suffix))
        elif self.action == 'pouring':
            # grasped_object_size = self.object_size
            #pour_object_size = self.convert_list_to_size(context['pour_object_size'])

            #new_height = (pour_object_size.z / 2) + (grasped_object_size.z / 2)
            #radius = (pour_object_size.x / 2) + (grasped_object_size.x / 2)
            radius = 0.0
            #self.object_size.z += new_height

            self.add_constraints_of_goal(GraspObject(goal_pose=self.goal_pose,
                                                     object_size=self.object_size,
                                                     reference_frame_alignment=self.reference_frame,
                                                     frontal_offset=radius,
                                                     from_above=self.from_above,
                                                     vertical_align=self.vertical_align,
                                                     root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     velocity=self.velocity,
                                                     weight=self.weight,
                                                     suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class GraspObject(ObjectGoal):
    def __init__(self,
                 goal_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 reference_frame_alignment: Optional[str] = 'base_link',
                 frontal_offset: Optional[float] = 0.0,
                 from_above: Optional[bool] = False,
                 vertical_align: Optional[bool] = False,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.goal_pose = goal_pose
        self.object_size = object_size
        self.reference_frame = reference_frame_alignment
        self.frontal_offset = frontal_offset
        self.from_above = from_above
        self.vertical_align = vertical_align
        self.root_link = self.world.search_for_link_name(root_link)
        self.root_str = self.root_link.short_name
        self.tip_link = self.try_to_get_link(expected=tip_link)
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = self.reference_frame

        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str

        self.goal_vertical_axis = Vector3Stamped()
        self.goal_vertical_axis.header.frame_id = self.reference_frame

        self.tip_vertical_axis = Vector3Stamped()
        self.tip_vertical_axis.header.frame_id = self.tip_str

        root_goal_point = PointStamped()
        root_goal_point.header.frame_id = self.goal_pose.header.frame_id
        root_goal_point.point = self.goal_pose.pose.position

        self.reference_link = self.world.search_for_link_name(self.reference_frame)

        self.goal_point = self.transform_msg(self.reference_link, root_goal_point)

        if self.from_above:
            # Grasp at the upper edge of the object
            #  self.goal_point.point.z += self.object_size.z / 2

            self.goal_vertical_axis.vector.x = 1
            self.goal_frontal_axis.vector.z = -1

        else:

            # Temporary solution to not be bothered with vertical grasping
            # self.bar_axis.vector = self.set_grasp_axis(self.object_size, maximum=True)
            self.goal_vertical_axis.vector.z = 1
            self.goal_frontal_axis.vector.x = 1

            self.goal_point.point.x += frontal_offset
            self.goal_point.point.z -= 0.01

        if self.vertical_align:
            self.tip_vertical_axis.vector.y = 1
        else:
            self.tip_vertical_axis.vector.x = 1

        self.tip_frontal_axis.vector.z = 1

        # Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point,
                                                       reference_velocity=self.velocity,
                                                       weight=self.weight,
                                                       suffix=self.suffix))

        # Align vertical
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_vertical_axis,
                                                 tip_normal=self.tip_vertical_axis,
                                                 reference_velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

        # Align frontal
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 reference_velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

        self.add_constraints_of_goal(NonRotationGoal(tip_link='base_link',
                                                     weight=self.weight,
                                                     suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def set_grasp_axis(self, axes: Vector3,
                       maximum: Optional[bool] = False):

        axes_array = [axes.x, axes.y, axes.z]

        sorting_values = axes_array.copy()
        sorting_values.sort(reverse=maximum)

        index_sorted_values = []
        for e in sorting_values:
            index_sorted_values.append(axes_array.index(e))

        grasp_vector = Vector3()
        if index_sorted_values[0] == 0:
            grasp_vector.x = 1
        elif index_sorted_values[0] == 1:
            grasp_vector.y = 1
        else:
            grasp_vector.z = 1

        return grasp_vector


class LiftObject(ObjectGoal):
    def __init__(self,
                 object_name: Optional[str] = '',
                 lifting: Optional[float] = 0.02,
                 root_link: Optional[str] = 'base_link',
                 tip_link: Optional[str] = 'hand_palm_link',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.object_name = object_name
        self.lifting_distance = lifting
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.try_to_get_link(expected=tip_link)
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name

        # Lifting
        start_point_tip = PoseStamped()
        start_point_tip.header.frame_id = self.tip_str

        goal_point_base = self.transform_msg(self.base_link, start_point_tip)

        goal_point_base.pose.position.z += self.lifting_distance

        goal_point_tip = self.transform_msg(self.tip_link, goal_point_base)

        self.goal_point = deepcopy(goal_point_tip)

        self.add_constraints_of_goal(NonRotationGoal(tip_link=self.tip_str,
                                                     weight=self.weight,
                                                     suffix=self.suffix))

        self.root_T_tip_start = self.world.compute_fk_np(self.root_link, self.tip_link)
        self.start_tip_T_current_tip = np.eye(4)

    def make_constraints(self):
        start_tip_T_current_tip = w.TransMatrix(self.get_parameter_as_symbolic_expression('start_tip_T_current_tip'))
        root_T_tip = self.get_fk(self.root_link, self.tip_link)

        t_T_g = w.TransMatrix(self.goal_point)
        r_T_tip_eval = w.TransMatrix(self.god_map.evaluate_expr(root_T_tip))

        root_T_goal = r_T_tip_eval.dot(start_tip_T_current_tip).dot(t_T_g)

        r_P_g = root_T_goal.to_position()
        r_P_c = root_T_tip.to_position()

        self.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.velocity,
                                        weight=self.weight)

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}{self.object_name}/{self.root_str}/{self.tip_str}_suffix:{self.suffix}'

    def update_params(self):
        root_T_tip_current = self.world.compute_fk_np(self.root_link, self.tip_link)
        self.start_tip_T_current_tip = np.dot(inverse_frame(self.root_T_tip_start), root_T_tip_current)


class Retracting(ObjectGoal):
    def __init__(self,
                 object_name: Optional[str] = '',
                 distance: Optional[float] = 0.2,
                 reference_frame: Optional[str] = 'base_link',
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Retract the tip link from the current position by the given distance.


        """

        super().__init__()

        self.object_name = object_name
        self.distance = distance
        self.reference_frame = self.try_to_get_link(expected=reference_frame)
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.try_to_get_link(expected=tip_link)
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        hand_frames = ['hand_gripper_tool_frame', 'hand_palm_link']
        base_frames = ['base_link']

        tip_P_start = PoseStamped()
        tip_P_start.header.frame_id = self.tip_str

        reference_P_start = self.transform_msg(self.reference_frame, tip_P_start)

        if self.reference_frame.short_name in hand_frames:
            reference_P_start.pose.position.z -= self.distance

        elif self.reference_frame.short_name in base_frames:
            reference_P_start.pose.position.x -= self.distance

        self.goal_point = self.transform_msg(self.tip_link, reference_P_start)
        self.root_T_tip_start = self.world.compute_fk_np(self.root_link, self.tip_link)

        self.start_tip_T_current_tip = np.eye(4)

        self.add_constraints_of_goal(NonRotationGoal(tip_link='base_link',
                                                     weight=self.weight,
                                                     suffix=self.suffix))

    def make_constraints(self):

        start_tip_T_current_tip = w.TransMatrix(self.get_parameter_as_symbolic_expression('start_tip_T_current_tip'))
        root_T_tip = self.get_fk(self.root_link, self.tip_link)

        t_T_g = w.TransMatrix(self.goal_point)
        r_T_tip_eval = w.TransMatrix(self.god_map.evaluate_expr(root_T_tip))

        root_T_goal = r_T_tip_eval.dot(start_tip_T_current_tip).dot(t_T_g)

        r_P_g = root_T_goal.to_position()
        r_P_c = root_T_tip.to_position()

        self.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.velocity,
                                        weight=self.weight)

        r_R_g = w.RotationMatrix(self.goal_point)
        r_R_c = self.get_fk(self.root_link, self.tip_link).to_rotation()
        c_R_r_eval = self.get_fk_evaluated(self.tip_link, self.root_link).to_rotation()
        # self.add_debug_expr('trans', w.norm(r_P_c))
        self.add_rotation_goal_constraints(frame_R_current=r_R_c,
                                           frame_R_goal=r_R_g,
                                           current_R_frame_eval=c_R_r_eval,
                                           reference_velocity=self.velocity,
                                           weight=self.weight)

        #r_R_c = root_T_tip.to_rotation()
        #r_R_g = root_T_goal.to_rotation()
        #r_R_g.reference_frame = self.root_link
        #self.r_R_c_eval = self.get_fk_evaluated(self.root_link, self.tip_link).to_rotation()

        #self.add_rotation_goal_constraints(frame_R_current=r_R_c,
        #                                   frame_R_goal=r_R_g,
        #                                   current_R_frame_eval=self.r_R_c_eval,
        #                                   reference_velocity=self.velocity,
        #                                   weight=self.weight
        #                                   )


    def update_params(self):
        root_T_tip_current = self.world.compute_fk_np(self.root_link, self.tip_link)
        self.start_tip_T_current_tip = np.dot(inverse_frame(self.root_T_tip_start), root_T_tip_current)
        self.r_R_c_eval = self.get_fk_evaluated(self.root_link, self.tip_link).to_rotation()

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class AlignHeight(ObjectGoal):
    def __init__(self,
                 context,
                 object_name: Optional[str] = '',
                 goal_pose: Optional[PoseStamped] = None,
                 object_height: Optional[float] = 0.0,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Align the tip link with the given goal_pose to prepare for further action (e.g. grasping or placing)

        :param object_name: name of the object if added to world
        :param goal_pose: final destination pose
        :param object_height: height of the target object
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain
        :param velocity: m/s
        :param weight: default WEIGHT_ABOVE_CA
        :param suffix: additional naming to avoid goals with same name (used for SequenceGoals)
        """

        super().__init__()

        self.object_name = object_name

        # Get object from name
        if goal_pose is None:
            goal_pose, object_size = self.get_object_by_name(self.object_name)

            object_height = object_size.z

        self.goal_pose = goal_pose
        self.object_height = object_height
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.try_to_get_link(expected=tip_link)
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.from_above = False

        if 'from_above' in context:
            self.from_above = context['from_above']

        self.reference_str = self.root_str
        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name

        # CartesianPosition
        goal_point = PointStamped()
        goal_point.header.frame_id = self.goal_pose.header.frame_id
        goal_point.point = self.goal_pose.pose.position

        base_to_tip = self.world.compute_fk_pose(self.base_link, self.tip_link)

        offset = 0.02
        base_goal_point = self.transform_msg(self.base_link, goal_point)
        base_goal_point.point.x = base_to_tip.pose.position.x
        base_goal_point.point.z += (self.object_height / 2) + offset

        if self.from_above:
            base_goal_point.point.z += 0.05

            # Tip facing downwards
            base_V_g = Vector3Stamped()
            base_V_g.header.frame_id = self.base_str
            base_V_g.vector.z = -1

            tip_V_g = Vector3Stamped()
            tip_V_g.header.frame_id = self.tip_str
            tip_V_g.vector.z = 1

            self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     goal_normal=base_V_g,
                                                     tip_normal=tip_V_g))

            base_V_x = Vector3Stamped()
            base_V_x.header.frame_id = self.base_str
            base_V_x.vector.x = 1

            tip_V_x = Vector3Stamped()
            tip_V_x.header.frame_id = self.tip_str
            tip_V_x.vector.x = 1

            self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     goal_normal=base_V_x,
                                                     tip_normal=tip_V_x))

        else:
            # Tip facing frontal
            self.add_constraints_of_goal(NonRotationGoal(tip_link=self.tip_str,
                                                         weight=self.weight,
                                                         suffix=self.suffix))

        self.add_constraints_of_goal(NonRotationGoal(tip_link=self.base_str,
                                                     weight=self.weight,
                                                     suffix=self.suffix))

        self.goal_point = self.transform_msg(self.tip_link, base_goal_point)

        # Align height
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point,
                                                       reference_velocity=self.velocity,
                                                       weight=self.weight,
                                                       suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class Placing(ForceSensorGoal):
    def __init__(self,
                 context,
                 goal_pose: PoseStamped,
                 velocity: Optional[float] = 0.025,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):

        self.goal_pose = goal_pose
        self.from_above = False
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        if 'from_above' in context:
            self.from_above = context['from_above']

        super().__init__()

        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name

        self.add_constraints_of_goal(GraspObject(goal_pose=self.goal_pose,
                                                 root_link=self.base_str,
                                                 from_above=self.from_above,
                                                 velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def goal_cancel_condition(self) -> [(str, str, w.Expression)]:
        if self.from_above:
            z_force_threshold = w.Expression(0.0)
            z_force_condition = ['z_force', '>=', z_force_threshold]

            y_torque_threshold = w.Expression(-0.15)
            y_torque_condition = ['y_torque', '<=', y_torque_threshold]

            expressions = [z_force_condition, y_torque_condition]
        else:
            x_force_threshold = w.Expression(0.0)
            x_force_condition = ['x_force', '<=', x_force_threshold]

            y_torque_threshold = w.Expression(0.15)
            y_torque_condition = ['y_torque', '>=', y_torque_threshold]

            expressions = [x_force_condition, y_torque_condition]

        return expressions

    def recovery(self) -> Dict:
        joint_states = {'arm_lift_joint': 0.03}

        return joint_states


class Tilting(Goal):
    def __init__(self,
                 tilt_direction: Optional[str] = None,
                 tilt_angle: Optional[float] = None,
                 tip_link: Optional[str] = 'wrist_roll_joint',
                 suffix: Optional[str] = ''):
        super().__init__()

        max_angle = -2.0

        if tilt_angle is None:
            tilt_angle = max_angle

        if tilt_direction == 'right':
            tilt_angle = abs(tilt_angle)
        else:
            tilt_angle = abs(tilt_angle) * -1

        self.wrist_state = tilt_angle
        self.tip_link = tip_link
        self.suffix = suffix

        self.goal_state = {'wrist_roll_joint': self.wrist_state,
                           'arm_roll_joint': 0.0}

        self.add_constraints_of_goal(JointPositionList(goal_state=self.goal_state,
                                                       suffix=self.suffix))

        '''self.add_constraints_of_goal(JointPosition(joint_name=self.tip_link,
                                                   goal=self.tilt_angle,
                                                   max_velocity=100))'''

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class TakePose(Goal):

    def __init__(self,
                 pose_keyword: Optional[str] = None,
                 head_pan_joint: Optional[float] = None,
                 head_tilt_joint: Optional[float] = None,
                 arm_lift_joint: Optional[float] = None,
                 arm_flex_joint: Optional[float] = None,
                 arm_roll_joint: Optional[float] = None,
                 wrist_flex_joint: Optional[float] = None,
                 wrist_roll_joint: Optional[float] = None,
                 suffix: Optional[str] = ''):
        super().__init__()

        variables = locals()
        joints = [x for x in variables if 'joint' in x]
        given_joint_positions = {val: variables[val] for val in joints if variables[val] is not None}

        if pose_keyword is None:

            current_joint_positions = {val: self.world.state.get(self.world.search_for_joint_name(val)).position for val
                                       in joints}

            joint_states = current_joint_positions

        else:
            if pose_keyword == 'park':
                head_pan_joint = 0.0
                head_tilt_joint = 0.0
                arm_lift_joint = 0.0
                arm_flex_joint = 0.0
                arm_roll_joint = -1.5
                wrist_flex_joint = -1.5
                wrist_roll_joint = 0.0

            elif pose_keyword == 'perceive':
                head_pan_joint = 0.0
                head_tilt_joint = -0.65
                arm_lift_joint = 0.25
                arm_flex_joint = 0.0
                arm_roll_joint = -1.5
                wrist_flex_joint = -1.5
                wrist_roll_joint = 0.0

            elif pose_keyword == 'assistance':
                head_pan_joint = 0.0
                head_tilt_joint = 0.0
                arm_lift_joint = 0.0
                arm_flex_joint = 0.0
                arm_roll_joint = -1.5
                wrist_flex_joint = -1.5
                wrist_roll_joint = 1.6

            elif pose_keyword == 'pre_align_height':
                head_pan_joint = 0.0
                head_tilt_joint = 0.0
                arm_lift_joint = 0.0
                arm_flex_joint = 0.0
                arm_roll_joint = 0.0
                wrist_flex_joint = -1.5
                wrist_roll_joint = 0.0

            elif pose_keyword == 'carry':
                head_pan_joint = 0.0
                head_tilt_joint = -0.65
                arm_lift_joint = 0.0
                arm_flex_joint = -0.43
                arm_roll_joint = 0.0
                wrist_flex_joint = -1.17
                wrist_roll_joint = 0.0

            elif pose_keyword == 'test':
                head_pan_joint = 0.0
                head_tilt_joint = 0.0
                arm_lift_joint = 0.38
                arm_flex_joint = -1.44
                arm_roll_joint = 0.0
                wrist_flex_joint = -0.19
                wrist_roll_joint = 0.0

            else:
                loginfo(f'{pose_keyword} is not a valid pose')
                return

            joint_states = {
                'head_pan_joint': head_pan_joint,
                'head_tilt_joint': head_tilt_joint,
                'arm_lift_joint': arm_lift_joint,
                'arm_flex_joint': arm_flex_joint,
                'arm_roll_joint': arm_roll_joint,
                'wrist_flex_joint': wrist_flex_joint,
                'wrist_roll_joint': wrist_roll_joint}

        for joint_name in given_joint_positions:
            joint_states[joint_name] = given_joint_positions[joint_name]

        self.goal_state = joint_states
        self.suffix = suffix

        self.add_constraints_of_goal(JointPositionList(goal_state=self.goal_state,
                                                       suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class Mixing(Goal):

    def __init__(self,
                 center: PointStamped,
                 radius: float,
                 scale: float,
                 mixing_time: Optional[float] = 60,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.1,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.god_map.set_data(identifier.max_trajectory_length, (mixing_time * scale) + 1)

        self.center = self.transform_msg(self.world.root_link_name, center)
        self.radius = radius
        self.scale = scale
        self.mixing_time = mixing_time
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.world.search_for_link_name(tip_link)
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.add_constraints_of_goal(NonRotationGoal(tip_link='base_link'))

    def make_constraints(self):
        map_T_bf = self.get_fk(self.root_link, self.tip_link)
        t = self.god_map.to_expr(identifier.time) * self.scale
        t = w.min(t, self.mixing_time * self.scale)
        x = w.cos(t) * self.radius
        y = w.sin(t) * self.radius
        map_P_center = w.Point3(self.center)
        map_T_center = w.TransMatrix.from_point_rotation_matrix(map_P_center)
        center_V_center_to_bf_goal = w.Vector3((-x, -y, 0))
        map_V_bf_to_center = map_T_center.dot(center_V_center_to_bf_goal)
        bf_V_y = w.Vector3((0, 1, 0))
        map_V_y = map_T_bf.dot(bf_V_y)
        map_V_y.vis_frame = self.tip_link
        map_V_bf_to_center.vis_frame = self.tip_link
        map_V_y.scale(1)
        map_V_bf_to_center.scale(1)

        center_P_bf_goal = w.Point3((x, y, 0))
        map_P_bf_goal = map_T_center.dot(center_P_bf_goal)
        map_P_bf = map_T_bf.to_position()

        self.add_point_goal_constraints(frame_P_current=map_P_bf,
                                        frame_P_goal=map_P_bf_goal,
                                        reference_velocity=self.velocity,
                                        weight=self.weight,
                                        name='position')

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class NonRotationGoal(Goal):
    def __init__(self,
                 tip_link: str,
                 weight: float = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.tip_link = tip_link
        self.weight = weight
        self.suffix = suffix

        zero_quaternion = Quaternion(x=0, y=0, z=0, w=1)
        tip_orientation = QuaternionStamped(quaternion=zero_quaternion)
        tip_orientation.header.frame_id = self.tip_link

        self.add_constraints_of_goal(CartesianOrientation(root_link='map',
                                                          tip_link=self.tip_link,
                                                          goal_orientation=tip_orientation,
                                                          weight=self.weight,
                                                          suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}{self.tip_link}_suffix:{self.suffix}'
