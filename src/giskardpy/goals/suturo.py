from copy import deepcopy
from pprint import pprint
from typing import Optional, List, Dict

import actionlib
import controller_manager_msgs
import numpy as np
import rospy
import trajectory_msgs
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped, Quaternion
from tmc_control_msgs.msg import GripperApplyEffortGoal, GripperApplyEffortAction
from trajectory_msgs.msg import JointTrajectoryPoint

import giskardpy.utils.tfwrapper as tf
from giskardpy import casadi_wrapper as w, identifier
from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPosition, CartesianOrientation
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA, ForceSensorGoal
from giskardpy.goals.joint_goals import JointPositionList
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.qp.constraint import EqualityConstraint
from giskardpy.utils import math
from giskardpy.utils.logging import loginfo, logwarn
from giskardpy.utils.math import inverse_frame
import math as m


class ObjectGoal(Goal):
    """
    Inherit from this class if the goal tries to get the object by name from the world
    """

    def get_object_by_name(self, object_name):
        try:
            loginfo('trying to get objects with name')

            object_link = self.world.get_link(object_name)
            # TODO: When object has no collision: set size to 0, 0, 0
            object_collisions = object_link.collisions
            if len(object_collisions) == 0:
                object_geometry = BoxGeometry(link_T_geometry=np.eye(4), depth=0, width=0, height=0, color=None)
            else:
                object_geometry: LinkGeometry = object_link.collisions[0]

            goal_pose = self.world.compute_fk_pose('map', object_name)

            loginfo(f'goal_pose by name: {goal_pose}')

            # Declare instance of geometry
            if isinstance(object_geometry, BoxGeometry):
                object_type = 'box'
                object_geometry: BoxGeometry = object_geometry
                # FIXE use expression instead of vector3, unless its really a vector
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

    def try_to_get_size_from_geometry(self,
                                      name: str,
                                      geometry: LinkGeometry,
                                      frame_fallback: str,
                                      size_fallback: Vector3):
        # FIXME check which functions are actually being used
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
                 motion_sequence: [Dict]):
        """
        Current solution to execute Goals in a sequence. The Goals will be executed one by one in the given order.

        :param motion_sequence: Future Dictionary to structure the goals with 'goal_type_seq': kwargs_seq
        :param goal_type_seq: List of Goals to execute. Send a List of goal names as strings, these will be parsed in ros_msg_to_goal
        :param kwargs_seq: List of Goal arguments. Needs to be sent as: [first_goal_arguments, second_goal_arguments, ...]
        """

        super().__init__()

        self.motion_sequence = motion_sequence

        self.current_goal_number = 0
        self.eq_weights = []
        self.goal_summary = []

        for index, goal in enumerate(self.motion_sequence):
            for current_goal, goal_args in goal.items():
                params = deepcopy(goal_args)
                params['suffix'] = index

                goal_instance: Goal = current_goal(**params)
                self.add_constraints_of_goal(goal_instance)

                self.goal_summary.append(goal_instance)

    def make_constraints(self):

        constraints: Dict[str, EqualityConstraint] = self._equality_constraints

        eq_constraint_suffix = [f'_suffix:{x}' for x in range(len(self.motion_sequence))]

        for goal_number, suffix_text in enumerate(eq_constraint_suffix):

            ordered_eq_constraints = [x[1] for x in constraints.items() if suffix_text in x[0]]

            eq_constraint_weights = [1] * len(ordered_eq_constraints)
            self.eq_weights.append(eq_constraint_weights)

            all_exprs = 1
            for eq_number, constraint in enumerate(ordered_eq_constraints):
                compiled = constraint.capped_error(self.sample_period).compile()
                s = self.god_map.to_symbol(
                    self._get_identifier() + ['evaluate_constraint_weight', (goal_number, eq_number, compiled)])

                expr = w.Expression(s)
                constraint.quadratic_weight = constraint.quadratic_weight * expr

                all_exprs = all_exprs * expr

            # self.add_debug_expr(self.goal_summary[goal_number].__str__(),
            #                    all_exprs)

            print(self.eq_weights[goal_number])

    def evaluate_constraint_weight(self, goal_number, eq_number, compiled_constraint):
        # TODO: Test if this works
        if goal_number != self.current_goal_number:
            return 0

        eq_constraint_error = compiled_constraint.fast_call(self.god_map.get_values(compiled_constraint.str_params))

        self.eq_weights[goal_number][eq_number] = abs(eq_constraint_error) > 0.001

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
    _gripper_apply_force_client = actionlib.SimpleActionClient('/hsrb/gripper_controller/grasp',
                                                               GripperApplyEffortAction)
    _gripper_controller = actionlib.SimpleActionClient('/hsrb/gripper_controller/follow_joint_trajectory',
                                                       FollowJointTrajectoryAction)

    def __init__(self,
                 gripper_state: str,
                 suffix=''):
        """
        Open / CLose Gripper.
        Current implementation is not final and might be replaced with a follow joint trajectory connection.

        :param gripper_state: keyword to state the gripper. Possible options: 'open', 'neutral', 'close'
        """

        super().__init__()

        self.suffix = suffix
        self.gripper_state = gripper_state

        if self.gripper_state == 'open':
            self.close_gripper_force(0.8)

        elif self.gripper_state == 'close':
            self.close_gripper_force(-0.8)

        elif self.gripper_state == 'neutral':

            self.set_gripper_joint_position(0.5)

    def close_gripper_force(self, force=0.8):
        """
        Closes the gripper with the given force.
        :param force: force to grasp with should be between 0.2 and 0.8 (N)
        :return: applied effort
        """
        rospy.loginfo("Closing gripper with force: {}".format(force))
        f = force  # max(min(0.8, force), 0.2)
        goal = GripperApplyEffortGoal()
        goal.effort = f
        self._gripper_apply_force_client.send_goal(goal)

    def set_gripper_joint_position(self, position):
        """
        Sets the gripper joint to the given  position
        :param position: goal position of the joint -0.105 to 1.239 rad
        :return: error_code of FollowJointTrajectoryResult
        """
        pos = max(min(1.239, position), -0.105)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [u'hand_motor_joint']
        p = JointTrajectoryPoint()
        p.positions = [pos]
        p.velocities = [0]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(1)
        goal.trajectory.points = [p]
        self._gripper_controller.send_goal(goal)

    def make_constraints(self):
        pass

    def trigger(self):
        # todo vllt kriegt man mit einem trigger call in einer expr es hin in seq goals zu öffnen
        # if irgendwas dann trigger
        return 0

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class Reaching(ObjectGoal):
    def __init__(self,
                 context,  # FIXME typehint for context
                 object_name: str = '',
                 object_shape: str = '',  # FIXME use optional only when none is allowed
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
        Examples for context action: grasping, placing or pouring

        :param context: Context of this goal. Contains information about action and the gripper alignment
        :param object_name: Name of the object to use. Optional as long as goal_pose and object_size are filled instead
        :param object_shape: Shape of the object to manipulate. Edit object size when having a sphere or cylinder
        :param goal_pose: Goal pose for the object. Alternative if no object name is given.
        :param object_size: Given object size. Alternative if no object name is given.
        :param root_link: Current root Link
        :param tip_link: Current tip link
        :param velocity: Desired velocity of this goal
        :param weight: weight of this goal
        :param suffix: Only relevant for SequenceGoal interns
        """
        # FIXME use message to define keywords
        super().__init__()
        self.context = context
        self.object_name = object_name
        self.object_shape = object_shape
        self.root_str = root_link
        self.tip_str = tip_link
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix
        # FIXME default for root link
        # self.world.groups[self.world.group_names[0]].root_link_name
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
            object_in_world = True

        else:
            try:
                self.world.search_for_link_name(goal_pose.header.frame_id)
                self.goal_pose = goal_pose
            except:
                logwarn(f'Couldn\'t find {goal_pose.header.frame_id}. Searching in tf.')
                self.goal_pose = tf.lookup_pose('map', goal_pose)

            self.object_size = object_size  # self.convert_list_to_size(object_size)
            self.reference_frame = 'base_link'

            object_in_world = False
            logwarn(f'Deprecated warning: Please add object to giskard and set object name.')

        if self.action == 'grasping':
            if self.object_shape == 'sphere' or self.object_shape == 'cylinder':
                radius = self.object_size.x
            else:

                # object_in_world = self.get_object_by_name(self.object_name) is not None

                if object_in_world:
                    radius = -0.04  # shelf
                    radius = - 0.02  # drawer

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
            # pour_object_size = self.convert_list_to_size(context['pour_object_size'])

            # new_height = (pour_object_size.z / 2) + (grasped_object_size.z / 2)
            # radius = (pour_object_size.x / 2) + (grasped_object_size.x / 2)
            radius = 0.0
            # self.object_size.z += new_height

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

        elif self.action == 'door-opening':
            radius = -0.02

            base_P_goal = self.transform_msg(self.world.search_for_link_name('base_link'), self.goal_pose)

            '''self.add_constraints_of_goal(GraspObject(goal_pose=base_P_goal,
                                                     object_size=self.object_size,
                                                     reference_frame_alignment=self.reference_frame,
                                                     frontal_offset=radius,
                                                     from_above=self.from_above,
                                                     vertical_align=self.vertical_align,
                                                     root_link=self.root_str,
                                                     tip_link=self.tip_str,
                                                     velocity=self.velocity / 2,
                                                     weight=self.weight,
                                                     suffix=self.suffix))'''
            self.add_constraints_of_goal(GraspCarefully(goal_pose=base_P_goal,
                                                        object_size=self.object_size,
                                                        reference_frame_alignment=self.reference_frame,
                                                        frontal_offset=radius,
                                                        from_above=self.from_above,
                                                        vertical_align=self.vertical_align,
                                                        root_link=self.root_str,
                                                        tip_link=self.tip_str,
                                                        velocity=self.velocity / 2,
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
                 reference_frame_alignment: Optional[str] = 'base_link',  # FIXME base link as default is unintuitive
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
        self.reference_frame = reference_frame_alignment
        self.frontal_offset = frontal_offset
        self.from_above = from_above
        self.vertical_align = vertical_align
        self.root_link = self.world.search_for_link_name(root_link)
        self.root_str = self.root_link.short_name
        self.tip_link = self.world.search_for_link_name(tip_link)
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
            # self.tip_vertical_axis.vector.y = -1

        self.tip_frontal_axis.vector.z = 1
        # temp donbot
        # self.tip_frontal_axis.vector.z = -1

        # Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point,
                                                       reference_velocity=self.velocity,
                                                       weight=self.weight,
                                                       suffix=self.suffix))

        # FIXME you can use orientation goal instead of two align planes
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

        self.add_constraints_of_goal(KeepRotationGoal(tip_link='base_link',
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


class VerticalMotion(ObjectGoal):
    def __init__(self,
                 context,
                 distance: Optional[float] = 0.02,
                 root_link: Optional[str] = 'base_link',
                 tip_link: Optional[str] = 'hand_palm_link',
                 velocity: Optional[float] = 0.2,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        """
        Move the tip link vertical according to the given context.
        """

        super().__init__()

        self.context = context
        self.distance = distance
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.world.search_for_link_name(tip_link)
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

        if context['action'] == 'grasping':
            goal_point_base.pose.position.z += self.distance
        if context['action'] == 'placing':
            goal_point_base.pose.position.z -= self.distance

        goal_point_tip = self.transform_msg(self.tip_link, goal_point_base)

        self.goal_point = deepcopy(goal_point_tip)

        self.add_constraints_of_goal(KeepRotationGoal(tip_link=self.tip_str,
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
        return f'{s}{self.context["action"]}/{self.root_str}/{self.tip_str}_suffix:{self.suffix}'

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
        The exact direction is based on the given reference frame.

        """
        super().__init__()

        self.object_name = object_name
        self.distance = distance
        self.reference_frame = self.world.search_for_link_name(reference_frame)
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.world.search_for_link_name(tip_link)
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        hand_frames = ['hand_gripper_tool_frame', 'hand_palm_link']

        tip_P_start = PoseStamped()
        tip_P_start.header.frame_id = self.tip_str

        reference_P_start = self.transform_msg(self.reference_frame, tip_P_start)

        if self.reference_frame.short_name in hand_frames:
            reference_P_start.pose.position.z -= self.distance

        else:
            reference_P_start.pose.position.x -= self.distance

        self.goal_point = self.transform_msg(self.tip_link, reference_P_start)
        self.root_T_tip_start = self.world.compute_fk_np(self.root_link, self.tip_link)

        self.start_tip_T_current_tip = np.eye(4)

        self.add_constraints_of_goal(KeepRotationGoal(tip_link='base_link',
                                                      weight=self.weight,
                                                      suffix=self.suffix))

        if 'base_link' not in self.tip_str:
            self.add_constraints_of_goal(KeepRotationGoal(tip_link=self.tip_str,
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

        # self.add_debug_expr('retract_position', w.norm(r_P_c))

        self.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.velocity,
                                        weight=self.weight)

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

        try:
            self.world.search_for_link_name(goal_pose.header.frame_id)
            self.goal_pose = goal_pose
        except:
            logwarn(f'Couldn\'t find {goal_pose.header.frame_id}. Searching in tf.')
            self.goal_pose = tf.lookup_pose('map', goal_pose)

        self.object_height = object_height
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.world.search_for_link_name(tip_link)
        self.root_str = self.root_link.short_name
        self.tip_str = self.tip_link.short_name
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        self.from_above = False

        if 'from_above' in context:
            self.from_above = context['from_above']

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
            self.add_constraints_of_goal(KeepRotationGoal(tip_link=self.tip_str,
                                                          weight=self.weight,
                                                          suffix=self.suffix))

        self.add_constraints_of_goal(KeepRotationGoal(tip_link=self.base_str,
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


class GraspCarefully(ForceSensorGoal):
    def __init__(self,
                 goal_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 reference_frame_alignment: Optional[str] = 'base_link',  # FIXME base link as default is unintuitive
                 frontal_offset: Optional[float] = 0.0,
                 from_above: Optional[bool] = False,
                 vertical_align: Optional[bool] = False,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.02,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.suffix = suffix

        self.add_constraints_of_goal(GraspObject(goal_pose=goal_pose,
                                                 object_size=object_size,
                                                 reference_frame_alignment=reference_frame_alignment,
                                                 frontal_offset=frontal_offset,
                                                 from_above=from_above,
                                                 vertical_align=vertical_align,
                                                 root_link=root_link,
                                                 tip_link=tip_link,
                                                 velocity=velocity,
                                                 weight=weight,
                                                 suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def goal_cancel_condition(self):
        force_threshold = 5.0

        expression = (lambda sensor_values:
                      (abs(sensor_values['x_force']) >= force_threshold) or
                      (abs(sensor_values['y_force']) >= force_threshold) or
                      (abs(sensor_values['z_force']) >= force_threshold))

        return expression

    def recovery(self) -> Dict:
        return {}


class Placing(ForceSensorGoal):
    def __init__(self,
                 context,
                 goal_pose: PoseStamped,
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 velocity: Optional[float] = 0.02,
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
                                                 tip_link=tip_link,
                                                 velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def goal_cancel_condition(self):

        if self.from_above:
            z_force_threshold = 0.0
            # z_force_condition = lambda sensor_values: sensor_values['z_force'] >= z_force_threshold

            y_torque_threshold = -0.15
            # y_torque_condition = lambda sensor_values: sensor_values['y_torque'] <= y_torque_threshold

            expression = (lambda sensor_values:
                          (sensor_values['x_force'] <= x_force_threshold) or
                          (sensor_values['y_torque'] >= y_torque_threshold))

        else:
            x_force_threshold = 0.0
            # x_force_condition = lambda sensor_values: (sensor_values['x_force'] <= x_force_threshold)

            y_torque_threshold = 0.15
            # y_torque_condition = lambda sensor_values: sensor_values['y_torque'] >= y_torque_threshold

            expression = (lambda sensor_values:
                          (sensor_values['x_force'] <= x_force_threshold) or
                          (sensor_values['y_torque'] >= y_torque_threshold))

        return expression

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
                wrist_roll_joint = -1.62

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

            # FIXME joint position list kann direkt gecalled werden, wenn cram ihn selbst definiert
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
                 mixing_time: Optional[float] = 20,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        super().__init__()

        self.weight = weight
        self.suffix = suffix

        target_speed = 1

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='wrist_roll_joint',
                                                                 joint_center=0.0,
                                                                 joint_range=0.9,
                                                                 trajectory_length=mixing_time,
                                                                 target_speed=target_speed,
                                                                 suffix=suffix))

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='wrist_flex_joint',
                                                                 joint_center=-1.3,
                                                                 joint_range=0.2,
                                                                 trajectory_length=mixing_time,
                                                                 target_speed=target_speed,
                                                                 suffix=suffix))

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='arm_roll_joint',
                                                                 joint_center=0.0,
                                                                 joint_range=0.1,
                                                                 trajectory_length=mixing_time,
                                                                 target_speed=target_speed,
                                                                 suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class JointRotationGoalContinuous(Goal):
    def __init__(self,
                 joint_name: str,
                 joint_center: float,
                 joint_range: float,
                 trajectory_length: Optional[float] = 20,
                 target_speed: Optional[float] = 1,
                 period_length: Optional[float] = 1.0,
                 suffix: Optional[str] = ''
                 ):
        super().__init__()
        self.joint = self.world.search_for_joint_name(joint_name)
        self.target_speed = target_speed
        self.trajectory_length = trajectory_length
        self.joint_center = joint_center
        self.joint_range = joint_range
        self.period_length = period_length
        self.suffix = suffix

    def make_constraints(self):
        time = self.traj_time_in_seconds()
        joint_position = self.get_joint_position_symbol(self.joint)

        joint_goal = self.joint_center + (w.cos(time * np.pi * self.period_length) * self.joint_range)

        self.add_debug_expr(f'{self.joint.short_name}_goal', joint_goal)
        self.add_debug_expr(f'{self.joint.short_name}_position', joint_position)

        self.add_position_constraint(expr_current=joint_position,
                                     expr_goal=joint_goal,
                                     reference_velocity=self.target_speed,
                                     weight=w.if_greater(time, self.trajectory_length, 0, WEIGHT_ABOVE_CA),
                                     name=self.joint.short_name)

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}{self.joint.short_name}_suffix:{self.suffix}'


class KeepRotationGoal(Goal):
    def __init__(self,
                 tip_link: str,
                 weight=WEIGHT_ABOVE_CA,
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


class PushButton(ForceSensorGoal):
    def __init__(self,
                 goal_pose: PoseStamped,
                 tip_link: Optional[str] = 'gripper_tool_frame',
                 velocity: Optional[float] = 0.01,
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: Optional[str] = ''):
        self.goal_pose = goal_pose
        self.from_above = False
        self.velocity = velocity
        self.weight = weight
        self.suffix = suffix

        super().__init__()

        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = self.base_link.short_name

        self.add_constraints_of_goal(GraspObject(goal_pose=self.goal_pose,
                                                 root_link=self.base_str,
                                                 from_above=self.from_above,
                                                 tip_link=tip_link,
                                                 velocity=self.velocity,
                                                 weight=self.weight,
                                                 suffix=self.suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def goal_cancel_condition(self):
        z_force_threshold = -1.0
        expression = lambda sensor_values: sensor_values['z_force'] <= z_force_threshold

        return expression

    # Move back after pushing the button
    def recovery(self) -> Dict:
        joint_states = {'odom_x': -0.05}

        return joint_states


class CheckForce(ForceSensorGoal):
    def __init__(self,
                 waiting_time=3):
        super().__init__()

        self.add_constraints_of_goal(JointRotationGoalContinuous(joint_name='head_pan_joint',
                                                                 joint_center=0.0,
                                                                 joint_range=0.1,
                                                                 trajectory_length=waiting_time,
                                                                 target_speed=0.02,
                                                                 suffix=''))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()

    def goal_cancel_condition(self):
        y_force_threshold = 0.0

        expression = lambda sensor_values: m.isclose(sensor_values['y_force'], y_force_threshold, abs_tol=0.3)

        return expression

    def recovery(self) -> Dict:
        pass