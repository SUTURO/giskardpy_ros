from copy import deepcopy
from typing import Optional, List, Dict

import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped
from trajectory_msgs.msg import JointTrajectoryPoint

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPositionStraight, CartesianPosition, CartesianOrientation
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA, ForceSensorGoal, NonMotionGoal
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPosition
from giskardpy.goals.pointing import Pointing
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.qp.constraint import EqualityConstraint
from giskardpy.utils.logging import loginfo, logwarn
from suturo_manipulation.gripper import Gripper

from giskardpy import casadi_wrapper as w, identifier


class ObjectGoal(Goal):
    def __init__(self):
        super().__init__()

    def get_object_by_name(self, object_name):
        try:
            loginfo('trying to get objects with name')

            object_link = self.world.get_link(object_name)
            object_geometry: LinkGeometry = object_link.collisions[0]

            # Get object
            object_pose = self.world.compute_fk_pose('map', object_name)

            loginfo(f'Object_pose by name: {object_pose}')

            # Declare instance of geometry
            if isinstance(object_geometry, BoxGeometry):
                object_type = 'box'
                object_geometry: BoxGeometry = object_geometry
                object_size = Vector3(object_geometry.width, object_geometry.depth, object_geometry.height)

            elif isinstance(object_geometry, CylinderGeometry):
                object_type = 'cylinder'
                object_geometry: CylinderGeometry = object_geometry
                object_size = None  # [object_geometry.height, object_geometry.radius]

            elif isinstance(object_geometry, SphereGeometry):
                object_type = 'sphere'
                object_geometry: SphereGeometry = object_geometry
                object_size = None  # [object_geometry.radius]

            else:
                raise Exception('Not supported geometry')

            loginfo(f'Got geometry: {object_type}')

            return object_pose, object_size, object_geometry

        except:
            loginfo('Could not get geometry from name')


class TestGoal(Goal):
    def __init__(self,
                 object_name: Optional[str] = '',
                 object_pose_1: Optional[PoseStamped] = None,
                 object_pose_2: Optional[PoseStamped] = None,
                 grasp_object: Optional[bool] = True,
                 lift_first: Optional[bool] = True):
        super().__init__()

        self.object_name = object_name
        self.object_pose_1 = object_pose_1
        self.object_pose_2 = object_pose_2
        self.grasp_object = grasp_object
        self.lift_first = lift_first

        print('Test Goal')

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class TestSequenceGoal(Goal):
    def __init__(self,
                 object_name='',
                 object_pose_1: PoseStamped = None,
                 object_pose_2: PoseStamped = None,
                 lift_first=True,
                 **kwargs):
        super().__init__()

        root_link = 'map'
        tip_link = 'hand_palm_link'
        self.root_link = self.world.search_for_link_name(root_link)
        self.tip_link = self.world.search_for_link_name(tip_link)
        self.root_str = str(root_link)
        self.tip_str = str(tip_link)

        self.object_name = object_name
        self.weight = WEIGHT_ABOVE_CA

        object_point_1 = PointStamped()
        object_point_1.header = object_pose_1.header
        object_point_1.point = object_pose_1.pose.position
        self.goal_point_1 = object_point_1

        object_point_2 = PointStamped()
        object_point_2.header = object_pose_2.header
        object_point_2.point = object_pose_2.pose.position
        self.goal_point_2 = object_point_2

        self.reference_velocity = 0.05

        self.goal_1_running = 1

        # first_goal_weight = self.weight*self.traj_time_in_seconds()
        # v = w.Vector3([0,0,1])
        # v.scale(weight/1000)
        # self.add_debug_expr('weight', v)
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point_1,
                                                       weight=self.weight))
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link='hand_gripper_tool_frame',
                                                       goal_point=self.goal_point_2,
                                                       weight=self.weight))

        print('working TestSequenceGoal')

        r_P_g = w.Point3(self.goal_point_1)
        r_P_c = self.get_fk(self.root_link, self.tip_link).to_position()
        s = self.god_map.to_symbol(self._get_identifier() + ['asdf', tuple()])

        v_current = w.Expression(r_P_c)
        v_goal = w.Expression(r_P_g)

        # If the distance between current and goal position is small: set weight to 0
        weight_one = s * w.Expression(self.weight * (w.euclidean_distance(v_current, v_goal) / 10))

        # If the weight of first goal reached 0, Increase this goals weight
        weight_two = w.Expression(WEIGHT_ABOVE_CA * (0 ** s))

        # self.add_debug_expr('trans', w.norm(r_P_c))
        # weight = self.weight*self.traj_time_in_seconds()

        goal_norm = Vector3Stamped()
        goal_norm.vector.z = 1

        tip_norm = Vector3Stamped()
        tip_norm.header.frame_id = self.tip_str
        tip_norm.vector.x = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=goal_norm,
                                                 tip_normal=tip_norm))

        # self._equality_constraints['asd']

        # self.add_constraints_of_goal(LiftObject(object_name=self.object_name, weight=self.changing_weight))

        # self.add_constraints_of_goal(Retracting(object_name=self.object_name, weight=self.changing_weight))

    def asdf(self):
        goal_1 = np.array((self.goal_point_1.point.x, self.goal_point_1.point.y, self.goal_point_1.point.z))

        current = self.world.compute_fk_np(self.root_link, self.tip_link)

        c_position = np.array((current[0][-1], current[1][-1], current[2][-1]))

        distance = np.linalg.norm(goal_1 - c_position)

        if distance < 0.03:
            self.goal_1_running = 0

        print(distance)

        return self.goal_1_running

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class SequenceGoal(Goal):
    def __init__(self,
                 goal_type_seq: List[Goal],
                 kwargs_seq: List[Dict],
                 # test: Dict,
                 **kwargs):
        super().__init__()

        self.goal_type_seq = goal_type_seq
        self.kwargs_seq = kwargs_seq

        self.current_goal = 0
        self.eq_weights = []

        self.current_endpoint = None

        goal_number = 0

        for goal, args in zip(self.goal_type_seq, kwargs_seq):

            params = deepcopy(args)
            params['tip_starting_position'] = self.current_endpoint
            params['suffix'] = goal_number

            g: Goal = goal(**params)

            mod = deepcopy(g.endpoint_modifier())

            self.add_constraints_of_goal(g)

            goal_number += 1

            if self.current_endpoint is None:
                self.current_endpoint = mod
                continue

            for tip_link, tip_endpoint in mod.items():
                if tip_link in self.current_endpoint.keys():
                    point = self.current_endpoint[tip_link]
                    point.point.x += tip_endpoint.point.x
                    point.point.y += tip_endpoint.point.y
                    point.point.z += tip_endpoint.point.z
                else:
                    self.current_endpoint[tip_link] = tip_endpoint

        print()


    def make_constraints(self):

        constraints: Dict[str, EqualityConstraint] = self._equality_constraints

        eq_constraint_suffix = [f'_suffix:{x}' for x in range(len(self.goal_type_seq))]

        values = constraints.items()
        for goal_number, suffix_text in enumerate(eq_constraint_suffix):

            ordered_eq_constraints = [x[1] for x in values if suffix_text in x[0]]

            eq_constraint_weights = [1] * len(ordered_eq_constraints)
            self.eq_weights.append(eq_constraint_weights)

            for eq_number, const in enumerate(ordered_eq_constraints):
                s = self.god_map.to_symbol(self._get_identifier() + ['asdf', (goal_number, eq_number, const.name)])

                expr = w.Expression(s)
                const.quadratic_weight = const.quadratic_weight * expr

        print()

    def asdf(self, goal_number, eq_number, goal_name):

        if goal_number != self.current_goal:
            return 0

        constraint: EqualityConstraint = self._equality_constraints.get(goal_name)

        sample = self.god_map.get_data(identifier=identifier.sample_period)

        compiled = constraint.capped_error(sample).compile()
        f = compiled.fast_call(self.god_map.get_values(compiled.str_params))

        if abs(f) < 0.001:
            self.eq_weights[goal_number][eq_number] = 0
        else:
            self.eq_weights[goal_number][eq_number] = 1

        if not any(self.eq_weights[goal_number]):
            self.current_goal += 1

            print('next goal')

            return 0

        print(f'{f} running on {goal_number} {eq_number}')

        return 1

    def __str__(self) -> str:
        return super().__str__()


class TestForceSensorGoal(ForceSensorGoal):
    def __init__(self,
                 object_pose: PoseStamped = None,
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


class SetPointing(Goal):
    def __init__(self,
                 goal_pose: PoseStamped,
                 root_link: str = 'map',
                 tip_link: str = 'hand_palm_link'):
        super().__init__()

        point_goal = PointStamped()
        point_goal.header.frame_id = goal_pose.header.frame_id
        point_goal.point = goal_pose.pose.position

        self.add_constraints_of_goal(Pointing(root_link=root_link,
                                              tip_link=tip_link,
                                              goal_point=point_goal))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class GraspObject(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 frontal_grasping: Optional[bool] = True,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 suffix: str = ''):
        """
        Determine the grasping perspective of the object
        """
        super().__init__()
        self.suffix = suffix
        self.object_name = object_name
        self.object_geometry = None
        self.root_link = root_link
        self.tip_link = tip_link
        self.frontal_grasping = frontal_grasping

        # Get object geometry from name
        if object_pose is None:
            self.object_pose, self.object_size, self.object_geometry = self.get_object_by_name(self.object_name)

        else:
            self.object_pose = object_pose
            self.object_size = object_size

            logwarn(f'Deprecated warning: Please add object to giskard and set object name.')

        # TODO: Add restriction: if object_pose.z + grasping difference + object_size/2 > 0.79: frontal grasping = False
        if self.object_size.z < 0.04:
            self.frontal_grasping = False

        if self.frontal_grasping:
            self.add_constraints_of_goal(GraspFrontal(object_name=self.object_name,
                                                      object_pose=self.object_pose,
                                                      object_size=self.object_size,
                                                      object_geometry=self.object_geometry,
                                                      root_link=self.root_link,
                                                      tip_link=self.tip_link,
                                                      suffix=suffix))
        else:
            self.add_constraints_of_goal(GraspAbove(object_name=self.object_name,
                                                    object_pose=self.object_pose,
                                                    object_size=self.object_size,
                                                    object_geometry=self.object_geometry,
                                                    root_link=self.root_link,
                                                    tip_link=self.tip_link,
                                                    suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class GraspAbove(Goal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 object_geometry: Optional[LinkGeometry] = None,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: str = ''):
        super().__init__()

        self.suffix = suffix

        # root link
        self.root = self.world.search_for_link_name(root_link, None)
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.search_for_link_name(tip_link, None)
        self.tip_str = str(self.tip)

        # Grasp at the upper edge of the object
        object_pose.pose.position.z += object_size.z / 2
        self.object_pose = object_pose

        root_goal_point = PointStamped()
        root_goal_point.header.frame_id = self.object_pose.header.frame_id
        root_goal_point.point = self.object_pose.pose.position

        if isinstance(object_geometry, BoxGeometry):
            self.object_size = Vector3(x=object_geometry.width, y=object_geometry.depth, z=object_geometry.height)

            reference_frame = object_name

        else:
            # Object not in giskard. Calculation will be less precise
            self.object_size = Vector3(x=object_size.x, y=object_size.y, z=object_size.z)

            reference_frame = 'base_link'
            reference_frame = 'map'

        # Frame/grasp difference
        frame_difference = 0.08

        grasping_difference = max(0.01, (frame_difference - (self.object_size.z / 2)))

        # Root -> Reference frame for hand_palm_link offset
        offset_tip_goal_point = self.transform_msg(reference_frame, root_goal_point)
        # offset_tip_goal_point.point.z = offset_tip_goal_point.point.z + grasping_difference

        bar_tolerance = 0.1

        if self.object_size.x >= self.object_size.y:
            tip_bar_vector = Vector3(x=1, y=0, z=0)
            bar_length = self.object_size.x
            print('using x')

        else:
            tip_bar_vector = Vector3(x=0, y=-1, z=0)
            bar_length = self.object_size.y
            print('using y')

        self.tip_vertical_axis = Vector3Stamped()
        self.tip_vertical_axis.header.frame_id = self.tip_str
        self.tip_vertical_axis.vector = tip_bar_vector

        # bar_axis
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = reference_frame
        self.bar_axis.vector = tip_bar_vector

        # bar_center
        self.bar_center_point = self.transform_msg(self.tip, offset_tip_goal_point)

        print(type(self.bar_center_point))

        # bar_length
        self.bar_length = bar_length * bar_tolerance

        # Align Planes
        # object axis horizontal/vertical
        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = reference_frame
        self.goal_frontal_axis.vector.z = -1

        # align z tip axis with object axis
        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str
        self.tip_frontal_axis.vector.z = 1

        '''self.add_constraints_of_goal(GraspBar(root_link=self.root_str,
                                              tip_link=self.tip_str,
                                              tip_grasp_axis=self.tip_vertical_axis,
                                              bar_center=self.bar_center_point,
                                              bar_axis=self.bar_axis,
                                              bar_length=self.bar_length))'''

        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.bar_center_point,
                                                       weight=weight,
                                                       suffix=suffix))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.bar_axis,
                                                 tip_normal=self.tip_vertical_axis,
                                                 weight=weight,
                                                 suffix=suffix))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 weight=weight,
                                                 suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class GraspFrontal(Goal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 object_geometry: Optional[LinkGeometry] = None,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: str = ''):
        """
        Move to a given position where a box can be grasped.

        :param object_pose: center position of the grasped object
        :param object_size: box size as Vector3 (x, y, z)
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain

        """
        super().__init__()

        self.suffix = suffix

        self.object_name_str = str(object_name)

        # root link
        self.root = self.world.search_for_link_name(root_link)
        self.root_str = str(self.root)

        # tip link
        try:
            self.tip = self.world.search_for_link_name(tip_link)
        except:
            self.tip = self.world.search_for_link_name('hand_palm_link')

        self.tip_str = str(self.tip)

        # Grasp slightly below the center of the object
        object_pose.pose.position.z = object_pose.pose.position.z - 0.01
        self.object_pose = object_pose

        self.weight = weight

        root_goal_point = PointStamped()
        root_goal_point.header.frame_id = self.object_pose.header.frame_id
        root_goal_point.point = self.object_pose.pose.position

        if isinstance(object_geometry, BoxGeometry):
            self.object_size = Vector3(x=object_geometry.width, y=object_geometry.depth, z=object_geometry.height)

            reference_frame = object_name

        else:
            # Object not in giskard. Calculation will be less precise
            self.object_size = Vector3(x=object_size.x, y=object_size.y, z=object_size.z)

            reference_frame = 'base_link'

        grasp_offset = min(0.07, self.object_size.x / 2)

        # tip_axis
        self.tip_vertical_axis = Vector3Stamped()
        self.tip_vertical_axis.header.frame_id = self.tip_str
        self.tip_vertical_axis.vector.x = 1

        # bar_center
        self.bar_center_point = self.transform_msg(reference_frame, root_goal_point)

        # self.bar_center_point.point.x += grasp_offset  # Grasp general
        # self.bar_center_point.point.x -= grasp_offset  # Grasp door handle

        # bar_axis
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = self.root_str

        # Temporary solution to not be bothered with vertical grasping
        # self.bar_axis.vector = self.set_grasp_axis(self.object_size, maximum=True)
        self.bar_axis.vector.z = 1

        # bar length
        self.bar_length = 0.0001

        # Align Planes
        # object axis horizontal/vertical
        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = reference_frame
        self.goal_frontal_axis.vector.x = 1

        # align z tip axis with object axis
        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str
        self.tip_frontal_axis.vector.z = 1

        hgtf = 'hand_gripper_tool_frame'

        self.add_constraints_of_goal(GraspBar(root_link=self.root_str,
                                              tip_link=self.tip_str,
                                              tip_grasp_axis=self.tip_vertical_axis,
                                              bar_center=self.bar_center_point,
                                              bar_axis=self.bar_axis,
                                              bar_length=self.bar_length,
                                              weight=self.weight,
                                              suffix=suffix))

        # Align frontal
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 weight=self.weight,
                                                 suffix=suffix))

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


class LiftObject(Goal):
    def __init__(self,
                 object_name: str,
                 lifting: float = 0.02,
                 root_link: str = 'map',
                 tip_link: str = 'hand_gripper_tool_frame',
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 reference_velocity: Optional[float] = None,
                 suffix: str = '',
                 tip_starting_position: Dict[str, PointStamped] = None):
        super().__init__()
        if reference_velocity is None:
            reference_velocity = 0.2

        if tip_starting_position is None:
            tip_starting_position = {}

        self.suffix = suffix
        self.object_name = object_name

        # root link
        self.root = self.world.search_for_link_name(root_link)
        self.root_str = str(self.root)

        # tip link
        try:
            self.tip = self.world.search_for_link_name(tip_link)
        except:
            hand_palm_link = 'hand_palm_link'
            self.tip = self.world.search_for_link_name(hand_palm_link)

            logwarn(f'Could not find {tip_link}. Fallback to {hand_palm_link}')

        self.tip_str = str(self.tip)

        self.lifting_distance = lifting
        self.weight = weight

        self.reference_velocity = reference_velocity
        self.tip_starting_position = tip_starting_position

        # Lifting
        goal_point = PointStamped()
        goal_point.header.frame_id = self.tip_str
        goal_point.point.x += self.lifting_distance

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
                                                 weight=self.weight,
                                                 suffix=suffix))




        '''self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=self.goal_point,
                                                       weight=self.weight,
                                                       suffix=suffix))'''

    def make_constraints(self):

        r_P_c = self.get_fk(self.root, self.tip).to_position()

        goal_transformed = self.transform_msg(self.root, self.goal_point)

        for tip_name, starting_position in self.tip_starting_position.items():
            new_start_tip = self.world.search_for_link_name(tip_name)

            goal_transformed = self.transform_msg(new_start_tip, self.goal_point)
            # transformed_position = self.transform_msg(self.root, self.tip_starting_position)

            goal_transformed.point.x = goal_transformed.point.x + starting_position.point.x
            goal_transformed.point.y = goal_transformed.point.y + starting_position.point.y
            goal_transformed.point.z = goal_transformed.point.z + starting_position.point.z

            goal_transformed = self.transform_msg(self.root, goal_transformed)

        r_P_g = w.Point3(goal_transformed)

        self.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.reference_velocity,
                                        weight=self.weight)

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}{self.object_name}/{self.root_str}/{self.tip_str}_suffix:{self.suffix}'

    def endpoint_modifier(self):
        goal_points = {self.tip.short_name: self.goal_point}

        return goal_points


class Retracting(Goal):
    def __init__(self,
                 object_name: str,
                 distance: Optional[float] = 0.2,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'base_link',
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: str = '',
                 reference_velocity: Optional[float] = None,
                 tip_starting_position: Dict[str, PointStamped] = None):
        super().__init__()

        if reference_velocity is None:
            reference_velocity = 0.2

        if tip_starting_position is None:
            tip_starting_position = {}

        self.suffix = suffix
        # root link
        self.root = self.world.search_for_link_name(root_link)
        self.root_str = str(self.root.short_name)

        # tip link
        self.tip = self.world.search_for_link_name(tip_link)
        self.tip_str = str(self.tip.short_name)

        self.weight = weight
        self.distance = distance

        self.reference_velocity = reference_velocity
        self.tip_starting_position = tip_starting_position

        goal_point = PointStamped()
        goal_point.header.frame_id = self.tip_str

        hand_frames = ['hand_gripper_tool_frame', 'hand_palm_link']
        base_frames = ['base_link']

        if self.tip_str in hand_frames:
            goal_point.point.z -= self.distance

        elif self.tip_str in base_frames:
            goal_point.point.x -= self.distance

        self.goal_point = goal_point

        print()


    def make_constraints(self):

        r_P_c = self.get_fk(self.root, self.tip).to_position()

        goal_transformed = self.transform_msg(self.root, self.goal_point)

        for tip_name, starting_position in self.tip_starting_position.items():
            new_start_tip = self.world.search_for_link_name(tip_name)

            goal_transformed = self.transform_msg(new_start_tip, self.goal_point)
            # transformed_position = self.transform_msg(self.root, self.tip_starting_position)

            goal_transformed.point.x = goal_transformed.point.x + starting_position.point.x
            goal_transformed.point.y = goal_transformed.point.y + starting_position.point.y
            goal_transformed.point.z = goal_transformed.point.z + starting_position.point.z

            goal_transformed = self.transform_msg(self.root, goal_transformed)

        r_P_g = w.Point3(goal_transformed)

        self.add_point_goal_constraints(frame_P_goal=r_P_g,
                                        frame_P_current=r_P_c,
                                        reference_velocity=self.reference_velocity,
                                        weight=self.weight)


    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def endpoint_modifier(self):
        goal_points = {self.tip.short_name: self.goal_point}

        return goal_points



class AlignHeight(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 goal_pose: Optional[PoseStamped] = None,
                 object_height: float = None,
                 height_only: Optional[bool] = True,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight: Optional[float] = WEIGHT_ABOVE_CA,
                 suffix: str = ''):
        super().__init__()

        self.suffix = suffix

        # root link
        self.root = self.world.search_for_link_name(root_link)
        self.root_str = str(self.root)

        # tip link
        try:
            self.tip = self.world.search_for_link_name(tip_link)
        except:
            hand_palm_link = 'hand_palm_link'
            self.tip = self.world.search_for_link_name(hand_palm_link)

            logwarn(f'Could not find {tip_link}. Fallback to {hand_palm_link}')

        self.tip_str = str(self.tip)

        self.weight = weight

        # Get object geometry from name
        if goal_pose is None:
            goal_pose, self.object_size, _ = self.get_object_by_name(object_name)

            object_height = self.object_size.z

        self.object_pose = goal_pose
        self.object_height = object_height

        # CartesianPosition
        goal_point = PointStamped()
        goal_point.header.frame_id = self.object_pose.header.frame_id
        goal_point.point = self.object_pose.pose.position

        self.base_link = self.world.search_for_link_name('base_link')
        self.base_str = str(self.base_link)

        base_to_tip = self.world.compute_fk_pose(self.base_link, self.tip)

        base_goal_point = self.transform_msg(self.base_link, goal_point)
        base_goal_point.point.x = base_to_tip.pose.position.x
        base_goal_point.point.z = base_goal_point.point.z + (self.object_height / 2)
        if height_only:
            base_goal_point.point.y = 0

        # Align height
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=base_goal_point,
                                                       weight=self.weight,
                                                       suffix=suffix))

        hand_orientation = QuaternionStamped()
        hand_orientation.header.frame_id = self.tip_str
        hand_orientation.quaternion.x = 0
        hand_orientation.quaternion.y = 0
        hand_orientation.quaternion.z = 0
        hand_orientation.quaternion.w = 1

        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                          tip_link=self.tip_str,
                                                          goal_orientation=hand_orientation,
                                                          suffix=suffix))

        base_orientation = QuaternionStamped()
        base_orientation.header.frame_id = self.base_str
        base_orientation.quaternion.x = 0
        base_orientation.quaternion.y = 0
        base_orientation.quaternion.z = 0
        base_orientation.quaternion.w = 1

        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                          tip_link=self.base_str,
                                                          goal_orientation=base_orientation,
                                                          suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class PlaceObject(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 target_pose: PoseStamped,
                 object_height: float = None,
                 frontal: Optional[bool] = True,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight: Optional[float] = None,
                 velocity: Optional[float] = None,
                 suffix: str = ''):
        super().__init__()

        self.suffix = suffix
        if velocity is None:
            velocity = 0.2
        if weight is None:
            weight = WEIGHT_ABOVE_CA

        # root link
        self.root = self.world.search_for_link_name(root_link)
        self.root_str = str(self.root)

        # tip link
        try:
            self.tip = self.world.search_for_link_name(tip_link)
        except:
            hand_palm_link = 'hand_palm_link'
            self.tip = self.world.search_for_link_name(hand_palm_link)

            logwarn(f'Could not find {tip_link}. Fallback to {hand_palm_link}')

        self.tip_str = str(self.tip)

        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = 'base_link'
        self.goal_frontal_axis.vector.x = 1

        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str

        self.goal_vertical_axis = Vector3Stamped()
        self.goal_vertical_axis.header.frame_id = self.root_str
        self.goal_vertical_axis.vector.z = 1

        self.tip_vertical_axis = Vector3Stamped()
        self.tip_vertical_axis.header.frame_id = self.tip_str

        if frontal:
            self.tip_frontal_axis.vector.z = 1
            self.tip_vertical_axis.vector.x = 1
        else:
            self.tip_frontal_axis.vector.x = 1
            self.tip_vertical_axis.vector.z = -1

        self.goal_floor_pose = target_pose

        self.weight = weight

        # Get object geometry from name
        if object_height is None:
            _, self.object_size, _ = self.get_object_by_name(object_name)

            object_height = self.object_size.z

        self.object_height = object_height

        goal_point = PointStamped()
        goal_point.header.frame_id = self.goal_floor_pose.header.frame_id
        goal_point.point.x = self.goal_floor_pose.pose.position.x
        goal_point.point.y = self.goal_floor_pose.pose.position.y
        goal_point.point.z = self.goal_floor_pose.pose.position.z + (self.object_height / 2)

        orientation_base = QuaternionStamped()
        orientation_base.header.frame_id = 'base_link'
        orientation_base.quaternion.x = 0
        orientation_base.quaternion.y = 0
        orientation_base.quaternion.z = 0
        orientation_base.quaternion.w = 1

        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_str,
                                                          tip_link='base_link',
                                                          goal_orientation=orientation_base,
                                                          suffix=suffix))

        # Align with destination
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 weight=self.weight,
                                                 suffix=suffix))

        # Align vertical
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_vertical_axis,
                                                 tip_normal=self.tip_vertical_axis,
                                                 weight=self.weight,
                                                 suffix=suffix))

        # Move to Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=goal_point,
                                                       weight=self.weight,
                                                       reference_velocity=velocity,
                                                       suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'


class PlaceNeatly(ForceSensorGoal):
    def __init__(self,
                 target_pose: PoseStamped,
                 weight=WEIGHT_ABOVE_CA,
                 suffix: str = ''):
        super().__init__()

        self.suffix = suffix
        velocity = 0.05

        self.add_constraints_of_goal(PlaceObject(object_name='',
                                                 target_pose=target_pose,
                                                 object_height=0.0,
                                                 weight=weight,
                                                 velocity=velocity,
                                                 suffix=suffix))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'

    def goal_cancel_condition(self) -> [(str, str, w.Expression)]:
        x_force_threshold = w.Expression(0.0)
        x_force_condition = ['x_force', '<=', x_force_threshold]

        y_torque_threshold = w.Expression(0.15)
        y_torque_condition = ['y_torque', '>=', y_torque_threshold]

        expressions = [x_force_condition, y_torque_condition]

        return expressions

    @staticmethod
    def recovery() -> Dict:
        joint_states = {'arm_lift_joint': 0.03}

        return joint_states


class Tilting(Goal):
    def __init__(self,
                 direction: Optional[str] = None,
                 tilt_angle: Optional[float] = None,
                 suffix: str = ''):
        super().__init__()

        self.suffix = suffix
        tip_link = 'wrist_roll_joint'

        max_angle = -2.0

        if tilt_angle is None:
            tilt_angle = max_angle

        if direction == 'right':
            tilt_angle = abs(tilt_angle)

        self.add_constraints_of_goal(JointPosition(goal=tilt_angle,
                                                   joint_name=tip_link))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        s = super().__str__()
        return f'{s}_suffix:{self.suffix}'
