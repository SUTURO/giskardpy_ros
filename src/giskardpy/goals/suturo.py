from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPositionStraight, CartesianPosition, CartesianOrientation, \
    CartesianPose
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA, NonMotionGoal
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPosition
from giskardpy.goals.pointing import Pointing
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.tree.behaviors.suturo_monitor_force_sensor import MonitorForceSensor
from giskardpy.tree.garden import TreeManager
from giskardpy.utils.logging import loginfo, logwarn
from suturo_manipulation.gripper import Gripper

from giskardpy import casadi_wrapper as w, identifier


class ForceSensorGoal(Goal):
    """
    Inherit from this goal, if the goal should use the Force Sensor.
    """

    def __init__(self):
        super().__init__()

        tree: TreeManager = self.god_map.get_data(identifier.tree_manager)

        t = tree.get_node('Monitor_Force')

        print(t)

        tree.insert_node(MonitorForceSensor('Monitor_Force'), 'monitor execution', 2)

        tree.render()

        return

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


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
                 goal_name: str,
                 object_name: Optional[str] = '',
                 object_pose_1: Optional[PoseStamped] = None,
                 object_pose_2: Optional[PoseStamped] = None,
                 grasp_object: Optional[bool] = True,
                 lift_first: Optional[bool] = True):
        super().__init__()

        self.goal_name = goal_name
        self.object_name = object_name
        self.object_pose_1 = object_pose_1
        self.object_pose_2 = object_pose_2
        self.grasp_object = grasp_object
        self.lift_first = lift_first

        print('Test Goal')

        goal = globals()[self.goal_name](object_name=self.object_name,
                                         object_pose_1=self.object_pose_1,
                                         object_pose_2=self.object_pose_2,
                                         grasp_object=self.grasp_object,
                                         lift_first=self.lift_first)
        self.add_constraints_of_goal(goal)

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
        self.root_link = self.world.get_link_name(root_link)
        self.tip_link = self.world.get_link_name(tip_link)
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

        v = w.Vector3([0, 0, 1])
        v.vis_frame = 'iai_kitchen/living_room:room_corner_0_link'
        v.scale(weight_one)
        self.add_debug_expr('weight', v/100)


        goal_norm = Vector3Stamped()
        goal_norm.vector.z = 1

        tip_norm = Vector3Stamped()
        tip_norm.header.frame_id = self.tip_str
        tip_norm.vector.x = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=goal_norm,
                                                 tip_normal=tip_norm))

        # self._equality_constraints['asd'].

        # self.add_constraints_of_goal(LiftObject(object_name=self.object_name, weight=self.changing_weight))

        # self.add_constraints_of_goal(Retracting(object_name=self.object_name, weight=self.changing_weight))

    def __str__(self) -> str:
        return super().__str__()


class TestRotationGoal(Goal):

    def __init__(self,
                 object_name='',
                 object_pose: PoseStamped = None,
                 **kwargs):
        super().__init__()

        self.root_link = 'map'
        self.tip_link = 'hand_palm_link'

        origin_quaternion = object_pose.pose.orientation

        print('TestRotationGoal')

        # TODO: Orientation magic
        self.used_quaternion = QuaternionStamped()
        self.used_quaternion.quaternion = origin_quaternion


        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_link,
                                                          tip_link=self.tip_link,
                                                          goal_orientation=self.used_quaternion))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class TestForceSensorGoal(ForceSensorGoal):
    def __init__(self,
                 object_pose: PoseStamped = None,
                 **kwargs):
        super().__init__()

        self.add_constraints_of_goal(PlaceNeatly(target_pose=object_pose))

    def make_constraints(self):
        pass

    def __str__(self):
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
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 frontal_grasping=True):
        """
        Determine the grasping perspective of the object
        """
        super().__init__()
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
                                                      tip_link=self.tip_link))
        else:
            self.add_constraints_of_goal(GraspAbove(object_name=self.object_name,
                                                    object_pose=self.object_pose,
                                                    object_size=self.object_size,
                                                    object_geometry=self.object_geometry,
                                                    root_link=self.root_link,
                                                    tip_link=self.tip_link))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class GraspAbove(Goal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 object_geometry: Optional[LinkGeometry] = None,
                 root_link: Optional[str] = 'odom',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame'):
        super().__init__()

        # root link
        self.root = self.world.get_link_name(root_link, None)
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None)
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
                                                       goal_point=self.bar_center_point))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.bar_axis,
                                                 tip_normal=self.tip_vertical_axis))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class GraspFrontal(Goal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 object_geometry: Optional[LinkGeometry] = None,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight = WEIGHT_ABOVE_CA):
        """
        Move to a given position where a box can be grasped.

        :param object_pose: center position of the grasped object
        :param object_size: box size as Vector3 (x, y, z)
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain

        """
        super().__init__()
        # self.object_name = self.world.get_link_name(object_name, None)
        self.object_name_str = str(object_name)

        # root link
        self.root = self.world.search_for_link_name(root_link)
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.search_for_link_name(tip_link)
        self.tip_str = str(self.tip)

        # Grasp slightly below the center of the object
        object_pose.pose.position.z = object_pose.pose.position.z - 0.01
        self.object_pose = object_pose

        self.weight = weight

        root_goal_point = PointStamped()
        root_goal_point.header.frame_id = self.root_str
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
        self.bar_center_point.point.x += grasp_offset

        # bar_axis
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = self.root_str

        # Temprorary solution to not be bothered with vertical grasping
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

        self.add_constraints_of_goal(GraspBar(root_link=self.root_str,
                                              tip_link=self.tip_str,
                                              tip_grasp_axis=self.tip_vertical_axis,
                                              bar_center=self.bar_center_point,
                                              bar_axis=self.bar_axis,
                                              bar_length=self.bar_length,
                                              weight=self.weight))

        # Align frontal
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 weight=self.weight))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()

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
                 weight=WEIGHT_ABOVE_CA):
        super().__init__()

        # root link
        self.root = self.world.get_link_name(root_link, None)
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None)
        self.tip_str = str(self.tip)

        self.lifting_distance = lifting

        if isinstance(weight, float):
            self.weight = weight
        else:
            self.weight = weight()

        # Lifting
        goal_point = PointStamped()
        goal_point.header.frame_id = self.tip_str
        goal_point.point.x += self.lifting_distance

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
                                                 weight=self.weight))

        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=goal_point,
                                                       weight=self.weight))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class Retracting(Goal):
    def __init__(self,
                 object_name: str,
                 distance: Optional[float] = 0.2,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'base_link',
                 weight=WEIGHT_ABOVE_CA):
        super().__init__()

        # root link
        self.root = self.world.get_link_name(root_link, None).short_name
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None).short_name
        self.tip_str = str(self.tip)

        self.distance = distance

        if isinstance(weight, float):
            self.weight = weight
        else:
            self.weight = weight()


        goal_point = PointStamped()
        goal_point.header.frame_id = self.tip_str

        if self.tip_str == 'hand_gripper_tool_frame':
            goal_point.point.z -= self.distance

        elif self.tip_str == 'base_link':
            goal_point.point.x -= self.distance

        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=goal_point,
                                                       weight=self.weight))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class AlignHeight(ObjectGoal):
    def __init__(self,
                 object_name,
                 goal_pose: Optional[PoseStamped] = None,
                 object_height: float = None,
                 root_link: Optional[str] = 'base_link',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight = WEIGHT_ABOVE_CA,
                 frontal_grasping=True):
        super().__init__()

        # root link
        self.root = self.world.get_link_name(root_link, None)
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None)
        self.tip_str = str(self.tip)

        self.weight = weight

        # Get object geometry from name
        if goal_pose is None:
            goal_pose, self.object_size, _ = self.get_object_by_name(object_name)

            object_height = self.object_size.z

        self.object_pose = goal_pose
        self.object_height = object_height * 0.66

        self.frontal_grasping = frontal_grasping

        # CartesianPosition
        goal_point = PointStamped()
        goal_point.header.frame_id = self.object_pose.header.frame_id
        goal_point.point.x = self.object_pose.pose.position.x
        goal_point.point.y = self.object_pose.pose.position.y
        goal_point.point.z = self.object_pose.pose.position.z + self.object_height

        tip_goal_point = self.transform_msg(self.tip_str, goal_point)
        tip_goal_point.header.frame_id = self.tip_str
        tip_goal_point.point.y = 0
        tip_goal_point.point.z = 0

        # Align height
        self.add_constraints_of_goal(CartesianPositionStraight(root_link=self.root_str,
                                                               tip_link=self.tip_str,
                                                               goal_point=tip_goal_point,
                                                               weight=self.weight))

        # Align vertical
        goal_vertical_axis = Vector3Stamped()
        goal_vertical_axis.header.frame_id = self.root_str

        tip_vertical_axis = Vector3Stamped()
        tip_vertical_axis.header.frame_id = self.tip_str

        if self.frontal_grasping:
            goal_vertical_axis.vector.z = 1
            tip_vertical_axis.vector.x = 1
        else:
            goal_vertical_axis.vector.z = -1
            tip_vertical_axis.vector.z = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=goal_vertical_axis,
                                                 tip_normal=tip_vertical_axis,
                                                 weight=self.weight))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class PlaceObject(ObjectGoal):
    def __init__(self,
                 object_name: str,
                 target_pose: PoseStamped,
                 object_height: float = None,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_gripper_tool_frame',
                 weight = WEIGHT_ABOVE_CA):
        super().__init__()

        # root link
        self.root = self.world.get_link_name(root_link, None)
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None)
        self.tip_str = str(self.tip)

        self.goal_frontal_axis = Vector3Stamped()
        self.goal_frontal_axis.header.frame_id = "base_link"
        self.goal_frontal_axis.vector.x = 1

        self.tip_frontal_axis = Vector3Stamped()
        self.tip_frontal_axis.header.frame_id = self.tip_str
        self.tip_frontal_axis.vector.z = 1

        self.goal_vertical_axis = Vector3Stamped()
        self.goal_vertical_axis.header.frame_id = self.root_str
        self.goal_vertical_axis.vector.z = 1

        self.tip_vertical_axis = Vector3Stamped()
        self.tip_vertical_axis.header.frame_id = self.tip_str
        self.tip_vertical_axis.vector.x = 1

        self.goal_floor_pose = target_pose

        self.weight = weight

        # Get object geometry from name
        if object_height is None:
            _, self.object_size, _ = self.get_object_by_name(object_name)

            object_height = self.object_size.z

        self.object_height = object_height

        goal_point = PointStamped()
        goal_point.header.frame_id = self.root_str
        goal_point.point.x = self.goal_floor_pose.pose.position.x
        goal_point.point.y = self.goal_floor_pose.pose.position.y
        goal_point.point.z = self.goal_floor_pose.pose.position.z + (self.object_height / 2)

        # Align towards object
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_frontal_axis,
                                                 tip_normal=self.tip_frontal_axis,
                                                 weight=self.weight))

        # Align vertical
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.goal_vertical_axis,
                                                 tip_normal=self.tip_vertical_axis,
                                                 weight=self.weight))

        # Move to Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_str,
                                                       tip_link=self.tip_str,
                                                       goal_point=goal_point,
                                                       weight=self.weight))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class PlaceNeatly(ForceSensorGoal):
    def __init__(self,
                 target_pose: PoseStamped,
                 weight=WEIGHT_ABOVE_CA):
        super().__init__()

        self.add_constraints_of_goal(PlaceObject(object_name='',
                                                 target_pose=target_pose,
                                                 object_height=0.0,
                                                 weight=weight))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()




class SequenceGoal(Goal):
    def __init__(self):

        super().__init__()


    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class Tilting(Goal):
    def __init__(self,
                 object_name=''):
        super().__init__()

        g = -2.0
        name = 'wrist_roll_joint'

        """self.add_constraints_of_goal(JointPosition(goal=g,
                                                   joint_name=name,
                                                   ))"""

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()
