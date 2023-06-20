from copy import deepcopy
from typing import Optional, List, Dict

import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped, Quaternion, Pose, \
    Point

from giskardpy import casadi_wrapper as w, identifier
from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPosition, CartesianOrientation
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA, ForceSensorGoal
from giskardpy.goals.joint_goals import JointPosition
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.qp.constraint import EqualityConstraint
from giskardpy.utils.logging import loginfo, logwarn
from giskardpy.utils.math import inverse_frame
from suturo_manipulation.gripper import Gripper


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

            # Get object
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

    def try_to_get_link(self,
                        expected: str,
                        fallback: str):
        try:
            link = self.world.search_for_link_name(expected)
        except:
            link = self.world.search_for_link_name(fallback)

            logwarn(f'Could not find {expected}. Fallback to {fallback}')

        return link

    def try_to_get_size_from_geometry(self,
                                      name: str,
                                      geometry: LinkGeometry,
                                      frame_fallback: str,
                                      size_fallback: Vector3):

        if isinstance(geometry, BoxGeometry):
            object_size = Vector3(x=geometry.width, y=geometry.depth, z=geometry.height)

            reference_frame = name

        else:
            # Object not in giskard. Calculation will be less precise
            object_size = Vector3(x=size_fallback.x, y=size_fallback.y, z=size_fallback.z)

            reference_frame = frame_fallback

        return reference_frame, object_size


class TestForceSensorGoal(ForceSensorGoal):
    def __init__(self,
                 goal_pose: PoseStamped = None,
                 **kwargs):
        super().__init__()

        # self.add_constraints_of_goal(LiftObject(object_name=''))
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
                 goal_type_seq: List[Goal],
                 kwargs_seq: List[Dict],
                 # sequence_goals: Dict,
                 **kwargs):
        super().__init__()

        self.goal_type_seq = goal_type_seq
        self.kwargs_seq = kwargs_seq
        # self.sequence_goals = sequence_goals

        self.current_goal = 0
        self.eq_weights = []
        self.goal_summary = []

        # with dict:
        # for index, (goal, args) in enumerate(sequence_goals.items()):
        for index, (goal, args) in enumerate(zip(self.goal_type_seq, self.kwargs_seq)):
            params = deepcopy(args)
            params['suffix'] = index

            goal: Goal = goal(**params)
            self.add_constraints_of_goal(goal)

            self.goal_summary.append(goal)

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

        if goal_number != self.current_goal:
            return 0

        eq_constraint_error = compiled_constraint.fast_call(self.god_map.get_values(compiled_constraint.str_params))

        if abs(eq_constraint_error) < 0.001:
            self.eq_weights[goal_number][eq_number] = 0
        else:
            self.eq_weights[goal_number][eq_number] = 1

        goal_not_finished = any(self.eq_weights[goal_number])
        if goal_not_finished:
            return 1

        self.current_goal += 1

        if self.current_goal >= len(self.goal_summary):
            return 0

        self.goal_summary[self.current_goal].update_params()

        loginfo('next goal')

        return 0

    def __str__(self) -> str:
        return super().__str__()


class MoveGripper(Goal):
    def __init__(self,
                 open_gripper=True,
                 joint_position=1.0):
        """
        Open / CLose Gripper.
        Current implementation is not final and will be replaced with a follow joint trajectory connection.

        :param open_gripper: True to open gripper; False to close gripper.
        """

        super().__init__()
        self.g = Gripper(apply_force_action_server='/hsrb/gripper_controller/apply_force',
                         follow_joint_trajectory_server='/hsrb/gripper_controller/follow_joint_trajectory')

        if open_gripper:
            self.g.set_gripper_joint_position(joint_position)

        else:
            self.g.close_gripper_force(1)

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class Reaching(ObjectGoal):
    def __init__(self,
                 context,
                 object_name: str,
                 object_shape = '',
                 goal_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 from_above: Optional[bool] = False,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Determine the grasping perspective of the object
        """
        super().__init__()
        self.context = context
        self.object_name = object_name
        self.from_above = from_above
        self.root_str = root_link
        self.tip_str = tip_link
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        # Get object geometry from name
        if goal_pose is None:
            self.goal_pose, self.object_size = self.get_object_by_name(self.object_name)

            self.reference_frame = self.object_name

        else:
            self.goal_pose = goal_pose
            self.object_size = object_size  # self.convert_list_to_size(object_size)

            self.reference_frame = 'base_link'

            logwarn(f'Deprecated warning: Please add object to giskard and set object name.')

        # TODO: Add restriction: if goal_pose.z + grasping difference + object_size/2 > 0.79: frontal grasping = False
        if self.object_size.z < 0.04:
            self.from_above = True

        if context['action'] == 'grasping':
            if context['object_shape'] == 'sphere' or context['object_shape'] == 'cylinder':
                radius = self.object_size.x
            else:
                radius = 0.0

            if self.from_above:
                self.add_constraints_of_goal(GraspAbove(object_name=self.object_name,
                                                        goal_pose=self.goal_pose,
                                                        object_size=self.object_size,
                                                        reference_frame_alignment=self.reference_frame,
                                                        root_link=self.root_str,
                                                        tip_link=self.tip_str,
                                                        velocity=self.velocity,
                                                        weight=self.weight,
                                                        suffix=self.suffix))
            else:
                self.add_constraints_of_goal(GraspFrontal(object_name=self.object_name,
                                                          goal_pose=self.goal_pose,
                                                          object_size=self.object_size,
                                                          reference_frame_alignment=self.reference_frame,
                                                          root_link=self.root_str,
                                                          tip_link=self.tip_str,
                                                          velocity=self.velocity,
                                                          weight=self.weight,
                                                          suffix=self.suffix))
        elif context['action'] == 'placing':
            if context['object_shape'] == 'sphere' or context['object_shape'] == 'cylinder':
                radius = self.object_size.x
            else:
                radius = 0.0

            self.add_constraints_of_goal(PlaceObject(object_name=self.object_name,
                                                     goal_pose=self.goal_pose,
                                                     object_height=self.object_size.z,
                                                     radius=radius,
                                                     from_above=self.from_above,
                                                     root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     velocity=self.velocity,
                                                     weight=self.weight,
                                                     suffix=self.suffix))
        elif context['action'] == 'pouring':

            height = 0.0 # bowl_height / 2 + object_height/2
            radius = 0.0 # bowl_radius/2 + object_radius/2

            self.add_constraints_of_goal(PlaceObject(object_name=self.object_name,
                                                     goal_pose=self.goal_pose,
                                                     object_height=height,
                                                     radius=radius,
                                                     from_above=self.from_above,
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


class GraspAbove(ObjectGoal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 goal_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 reference_frame_alignment: Optional[str] = 'base_link',
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.object_name = object_name
        self.goal_pose = goal_pose
        self.object_size = object_size
        self.reference_frame = reference_frame_alignment
        self.root_link = self.world.search_for_link_name(root_link)
        self.root_str = self.root_link.short_name
        self.tip_link = self.try_to_get_link(expected=tip_link, fallback='hand_palm_link')
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        # Grasp at the upper edge of the object
        self.goal_pose.pose.position.z += self.object_size.z / 2

        root_goal_point = PointStamped()
        root_goal_point.header.frame_id = self.goal_pose.header.frame_id
        root_goal_point.point = self.goal_pose.pose.position

        '''reference_frame, self.object_size = self.try_to_get_size_from_geometry(name=self.object_name,
                                                                               geometry=self.object_geometry,
                                                                               frame_fallback='base_link',
                                                                               size_fallback=object_size)'''

        # Root -> Reference frame for hand_palm_link offset
        offset_tip_goal_point = self.transform_msg(self.reference_frame, root_goal_point)

        if self.object_size.x >= self.object_size.y:
            tip_bar_vector = Vector3(x=1, y=0, z=0)

        else:
            tip_bar_vector = Vector3(x=0, y=-1, z=0)

        self.tip_vertical_axis = Vector3Stamped(vector=tip_bar_vector)
        self.tip_vertical_axis.header.frame_id = self.tip_str

        # bar_axis
        self.bar_axis = Vector3Stamped(vector=tip_bar_vector)
        self.bar_axis.header.frame_id = self.reference_frame

        # bar_center
        self.goal_point = self.transform_msg(self.tip_link, offset_tip_goal_point)

        # Align Planes
        # object axis horizontal/vertical
        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = self.reference_frame
        self.goal_frontal_axis.vector.z = -1

        # align z tip axis with object axis
        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str
        self.tip_frontal_axis.vector.z = 1

        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point,
                                                       reference_velocity=self.velocity,
                                                       weight=self.weight,
                                                       suffix=self.suffix))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.bar_axis,
                                                 tip_normal=self.tip_vertical_axis,
                                                 reference_velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 reference_velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class GraspFrontal(ObjectGoal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 goal_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 reference_frame_alignment: Optional[str] = 'base_link',
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Move to a given position where a box can be grasped.

        :param goal_pose: center position of the grasped object
        :param object_size: box size as Vector3 (x, y, z)
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain

        """
        super().__init__()

        self.object_name = object_name
        self.goal_pose = goal_pose
        self.object_size = object_size
        self.reference_frame = reference_frame_alignment
        self.root_link = self.world.search_for_link_name(root_link)
        self.root_str = self.root_link.short_name
        self.tip_link = self.try_to_get_link(expected=tip_link, fallback='hand_palm_link')
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        object_in_world = self.get_object_by_name(self.object_name) is not None

        if object_in_world:
            '''reference_frame, self.object_size = self.try_to_get_size_from_geometry(name=self.object_name,
                                                                                   geometry=self.object_geometry,
                                                                                   frame_fallback='base_link',
                                                                                   size_fallback=object_size)'''
            grasp_offset = -0.04

        else:
            '''reference_frame = 'base_link'''

            grasp_offset = max(min(0.08, self.object_size.x / 2), 0.05)

        # Grasp slightly below the center of the object
        self.goal_pose.pose.position.z = self.goal_pose.pose.position.z - 0.01

        root_goal_point = PointStamped()
        root_goal_point.header.frame_id = self.goal_pose.header.frame_id
        root_goal_point.point = self.goal_pose.pose.position

        # tip_axis
        self.tip_vertical_axis = Vector3Stamped()
        self.tip_vertical_axis.header.frame_id = self.tip_str
        self.tip_vertical_axis.vector.x = 1

        # bar_center
        self.goal_point = self.transform_msg(self.reference_frame, root_goal_point)
        self.goal_point.point.x += grasp_offset

        # bar_axis
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = self.root_str

        # Temporary solution to not be bothered with vertical grasping
        # self.bar_axis.vector = self.set_grasp_axis(self.object_size, maximum=True)
        self.bar_axis.vector.z = 1

        # Align Planes
        # object axis horizontal/vertical
        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = self.reference_frame
        self.goal_frontal_axis.vector.x = 1

        # align z tip axis with object axis
        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str
        self.tip_frontal_axis.vector.z = 1

        # Align frontal
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 reference_velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

        # Align vertical
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.tip_vertical_axis,
                                                 tip_normal=self.bar_axis,
                                                 reference_velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

        # Target Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point,
                                                       reference_velocity=self.velocity,
                                                       weight=self.weight,
                                                       suffix=self.suffix))

        # Do not rotate base
        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name
        zero_quaternion = Quaternion(x=0, y=0, z=0, w=1)

        base_orientation = QuaternionStamped(quaternion=zero_quaternion)
        base_orientation.header.frame_id = self.base_str

        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                          tip_link=self.base_str,
                                                          goal_orientation=base_orientation,
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
                 object_name: str,
                 lifting: float = 0.02,
                 root_link: Optional[str] = 'base_link',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.object_name = object_name
        self.lifting_distance = lifting
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.try_to_get_link(expected=tip_link, fallback='hand_palm_link')
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        # Lifting
        goal_point = PoseStamped()
        goal_point.header.frame_id = self.tip_str
        goal_point.pose.position.x += self.lifting_distance

        self.goal_point = deepcopy(goal_point)

        # Align vertical
        goal_vertical_axis = Vector3Stamped()
        goal_vertical_axis.header.frame_id = self.root_str
        goal_vertical_axis.vector.z = 1

        tip_vertical_axis = Vector3Stamped()
        tip_vertical_axis.header.frame_id = self.tip_str
        tip_vertical_axis.vector.x = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=goal_vertical_axis,
                                                 tip_normal=tip_vertical_axis,
                                                 reference_velocity=self.velocity,
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

        '''c_R_r_eval = self.get_fk_evaluated(self.tip_link, self.root_link).to_rotation()

        self.add_rotation_goal_constraints(frame_R_goal=r_R_g,
                                           frame_R_current=r_R_c,
                                           current_R_frame_eval=c_R_r_eval,
                                           reference_velocity=,
                                           )'''

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}{self.object_name}/{self.root_str}/{self.tip_str}_suffix:{self.suffix}'

    def update_params(self):
        root_T_tip_current = self.world.compute_fk_np(self.root_link, self.tip_link)
        self.start_tip_T_current_tip = np.dot(inverse_frame(self.root_T_tip_start), root_T_tip_current)


class Retracting(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 distance: Optional[float] = 0.2,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'base_link',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Retract the tip link from the current position by the given distance.


        """

        super().__init__()

        self.object_name = object_name
        self.distance = distance
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.try_to_get_link(expected=tip_link, fallback='hand_palm_link')
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        hand_frames = ['hand_gripper_tool_frame', 'hand_palm_link']
        base_frames = ['base_link']

        tip_P_goal = PoseStamped()
        tip_P_goal.header.frame_id = self.tip_str

        if self.tip_link.short_name in hand_frames:
            tip_P_goal.pose.position.z -= self.distance

        elif self.tip_link.short_name in base_frames:
            tip_P_goal.pose.position.x -= self.distance

        self.goal_point = deepcopy(tip_P_goal)
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

        r_R_g = root_T_goal.to_rotation()
        r_R_c = root_T_tip.to_rotation()

        c_R_r_eval = self.get_fk_evaluated(self.tip_link, self.root_link).to_rotation()

        self.add_rotation_goal_constraints(frame_R_current=r_R_c,
                                           frame_R_goal=r_R_g,
                                           current_R_frame_eval=c_R_r_eval,
                                           reference_velocity=self.velocity,
                                           weight=self.weight)

    def update_params(self):
        root_T_tip_current = self.world.compute_fk_np(self.root_link, self.tip_link)
        self.start_tip_T_current_tip = np.dot(inverse_frame(self.root_T_tip_start), root_T_tip_current)

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class AlignHeight(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 goal_pose: Optional[PoseStamped] = None,
                 object_height: Optional[float] = 0.0,
                 from_above: Optional[bool] = False,
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
        self.from_above = from_above
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.try_to_get_link(expected=tip_link, fallback='hand_palm_link')
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

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
        base_goal_point.point.z = base_goal_point.point.z + (self.object_height / 2) + offset

        zero_quaternion = Quaternion(x=0, y=0, z=0, w=1)

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
            hand_orientation = QuaternionStamped(quaternion=zero_quaternion)
            hand_orientation.header.frame_id = self.tip_str

            self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                              tip_link=self.tip_str,
                                                              goal_orientation=hand_orientation,
                                                              weight=self.weight,
                                                              suffix=self.suffix))

        base_orientation = QuaternionStamped(quaternion=zero_quaternion)
        base_orientation.header.frame_id = self.base_str

        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                          tip_link=self.base_str,
                                                          goal_orientation=base_orientation,
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


class PlaceObject(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 goal_pose: PoseStamped,
                 object_height: Optional[float] = None,
                 radius: Optional[float] = 0.0,
                 from_above: Optional[bool] = False,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.object_name = object_name
        self.goal_pose = goal_pose

        # Get object geometry from name
        if object_height is None:
            try:
                _, self.object_size = self.get_object_by_name(self.object_name)
                object_height = self.object_size.z
            except:
                object_height = 0.0

        self.object_height = object_height
        self.radius = radius
        self.from_above = from_above
        self.root_link = self.world.search_for_link_name(root_link)
        self.root_str = self.root_link.short_name
        self.tip_link = self.try_to_get_link(expected=tip_link, fallback='hand_palm_link')
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name

        # Calculation
        z_offset = 0.02
        self.goal_pose.pose.position.z += (self.object_height / 2) + z_offset

        self.base_P_goal = self.transform_msg(self.base_link, self.goal_pose)
        self.base_P_goal.pose.position.x -= self.radius

        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = self.base_str
        self.goal_frontal_axis.vector.x = 1

        self.goal_vertical_axis = Vector3Stamped()
        self.goal_vertical_axis.header.frame_id = self.root_str
        self.goal_vertical_axis.vector.z = 1

        if self.from_above:
            tip_frontal_axis_vector = Vector3(x=1, y=0, z=0)
            tip_vertical_axis_vector = Vector3(x=0, y=0, z=-1)

            self.base_P_goal.pose.position.z += 0.05
        else:
            tip_frontal_axis_vector = Vector3(x=0, y=0, z=1)
            tip_vertical_axis_vector = Vector3(x=1, y=0, z=0)

        self.tip_vertical_axis = Vector3Stamped(vector=tip_vertical_axis_vector)
        self.tip_vertical_axis.header.frame_id = self.tip_str

        self.tip_frontal_axis = Vector3Stamped(vector=tip_frontal_axis_vector)
        self.tip_frontal_axis.header.frame_id = self.tip_str

        goal_point = PointStamped()
        goal_point.header.frame_id = self.base_P_goal.header.frame_id
        goal_point.point = self.base_P_goal.pose.position

        zero_quaternion = Quaternion(x=0, y=0, z=0, w=1)

        orientation_base = QuaternionStamped(quaternion=zero_quaternion)
        orientation_base.header.frame_id = self.base_str

        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                          tip_link=self.base_str,
                                                          goal_orientation=orientation_base,
                                                          weight=self.weight,
                                                          suffix=self.suffix))

        # Align with destination
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
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

        # Move to Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=goal_point,
                                                       reference_velocity=self.velocity,
                                                       weight=self.weight,
                                                       suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class PlaceNeatly(ForceSensorGoal):
    def __init__(self,
                 goal_pose: PoseStamped,
                 from_above: Optional[bool] = False,
                 velocity: Optional[float] = 0.025,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):

        self.goal_pose = goal_pose
        self.from_above = from_above
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        super().__init__()

        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name

        self.add_constraints_of_goal(PlaceObject(object_name='',
                                                 goal_pose=self.goal_pose,
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
                 direction: Optional[str] = None,
                 tilt_angle: Optional[float] = None,
                 tip_link: Optional[str] = 'wrist_roll_joint',
                 suffix: Optional[str] = ''):
        super().__init__()

        max_angle = -2.0

        if tilt_angle is None:
            tilt_angle = max_angle

        if direction == 'right':
            tilt_angle = abs(tilt_angle)

        self.tilt_angle = tilt_angle
        self.tip_link = tip_link
        self.suffix = suffix

        self.add_constraints_of_goal(JointPosition(goal=self.tilt_angle,
                                                   joint_name=self.tip_link))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'
