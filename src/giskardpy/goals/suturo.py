import asyncio
import logging
import time
from typing import Optional, List

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped
from tf.transformations import quaternion_from_matrix

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPositionStraight, CartesianPosition, CartesianOrientation, \
    CartesianPose
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPositionList
from giskardpy.goals.pointing import Pointing
from giskardpy.model.links import BoxGeometry, LinkGeometry, SphereGeometry, CylinderGeometry
from giskardpy.my_types import PrefixName
from giskardpy.utils.logging import loginfo, logwarn
from suturo_manipulation.gripper import Gripper

import giskardpy.utils.tfwrapper as tf
from giskardpy import casadi_wrapper as w, identifier

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry


class TestGoal(Goal):
    def __init__(self,
                 goal_name: str,
                 object_name: Optional[str] = '',
                 object_pose: Optional[PoseStamped] = None,
                 grasp_object: Optional[bool] = True,
                 lift_first: Optional[bool] = True):
        super().__init__()

        self.goal_name = goal_name
        self.object_name = object_name
        self.object_pose = object_pose
        self.grasp_object = grasp_object
        self.lift_first = lift_first

        print('Test Goal')

    def make_constraints(self):
        goal = globals()[self.goal_name](object_name=self.object_name,
                                         object_pose=self.object_pose,
                                         grasp_object=self.grasp_object,
                                         lift_first=self.lift_first)
        self.add_constraints_of_goal(goal)

    def __str__(self) -> str:
        return super().__str__()


class TestSequenceGoal(Goal):
    def __init__(self,
                 object_name='',
                 object_pose: PoseStamped = None,
                 lift_first=True,
                 **kwargs):
        super().__init__()

        self.root_link = 'map'
        self.tip_link = 'hand_palm_link'

        self.object_name = object_name

        self.var_number = lift_first
        self.changing_weight = lambda: self.inverse_weight()
        print(f'TestSequence: {self.changing_weight}')
        print(type(self.changing_weight))

        print('working TestSequenceGoal')

    def make_constraints(self):
        self.add_constraints_of_goal(LiftObject(object_name=self.object_name, weight=self.changing_weight))

        self.add_constraints_of_goal(Retracting(object_name=self.object_name, weight=self.changing_weight))

        # self.change_weight_to_zero_delay(delay=1)

    def __str__(self) -> str:
        return super().__str__()

    def inverse_weight(self):

        self.var_number = not self.var_number

        if self.var_number:
            return 1.0
        else:
            return 0.0


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

    def make_constraints(self):
        self.add_constraints_of_goal(CartesianOrientation(root_link=self.root_link,
                                                          tip_link=self.tip_link,
                                                          goal_orientation=self.used_quaternion))

    def __str__(self) -> str:
        return super().__str__()


class TestGripperGoal(Goal):

    def __init__(self,
                 object_name='',
                 object_pose: PoseStamped = None,
                 grasp_object: bool = True,
                 **kwargs):
        super().__init__()

        print('TestGripperGoal')

        g = Gripper(apply_force_action_server='/hsrb/gripper_controller/apply_force',
                    follow_joint_trajectory_server='/hsrb/gripper_controller/follow_joint_trajectory')

        if grasp_object:
            # g.close_gripper_force(1)
            print(g.object_in_gripper())
            if g.object_in_gripper():
                g.publish_object_in_gripper(object_frame_id=object_name, pose_stamped=object_pose, mode=0)

        else:
            g.set_gripper_joint_position(1)
            g.publish_object_in_gripper(object_frame_id=object_name, pose_stamped=object_pose, mode=1)

    def make_constraints(self):
        # Does not work
        # robot = hsrb_interface.Robot()
        # gripper = robot.try_get('gripper')
        pass

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


class GraspObject(Goal):
    def __init__(self,
                 object_name: str,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link',
                 offset: Optional[float] = 0.01,
                 frontal_grasping=True):
        """
        Determine the grasping perspective of the object
        """
        super().__init__()
        self.object_name = object_name
        self.object_pose = object_pose
        self.object_size = object_size
        self.object_geometry = None
        self.root_link = root_link
        self.tip_link = tip_link
        self.offset = offset
        self.frontal_grasping = frontal_grasping

        # Get object geometry from name
        if object_pose is None:
            self.object_pose, self.object_size, self.object_geometry = self.get_object_by_name(self.object_name)

        else:
            logwarn(f'Deprecated warning: Please add object to giskard and set object name.')

        # TODO: Implement check if grasping should be frontal or above

    def make_constraints(self):
        if self.frontal_grasping:
            self.add_constraints_of_goal(GraspFrontal(object_name=self.object_name,
                                                      object_pose=self.object_pose,
                                                      object_size=self.object_size,
                                                      object_geometry=self.object_geometry,
                                                      root_link=self.root_link,
                                                      tip_link=self.tip_link,
                                                      offset=self.offset))
        else:
            self.add_constraints_of_goal(GraspAbove(object_pose=self.object_pose,
                                                    object_size=self.object_size,
                                                    object_geometry=self.object_geometry,
                                                    root_link=self.root_link,
                                                    tip_link=self.tip_link,
                                                    offset=self.offset))

    def __str__(self) -> str:
        return super().__str__()

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

class GraspGoal(Goal):
    def __init__(self):
        pass

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class GraspAbove(Goal):
    def __init__(self,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 object_geometry: Optional[LinkGeometry] = None,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link',
                 offset: Optional[float] = 0.01):
        super().__init__()

        # root link
        self.root = self.world.get_link_name(root_link, None)
        loginfo(f'root_link: {self.root}')
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None)
        loginfo(f'tip_link: {self.tip}')
        self.tip_str = str(self.tip)

        # Grasp slightly below the center of the object
        # object_pose.pose.position.z = object_pose.pose.position.z - 0.02

        # Grasp at the upper edge of the object
        object_pose.pose.position.z += object_size.z / 2
        self.object_pose = object_pose


        root_P_box_point = PointStamped()
        root_P_box_point.header.frame_id = self.root_str
        root_P_box_point.point = self.object_pose.pose.position

        # root -> tip tranfsormation
        self.tip_P_goal_point = self.transform_msg(self.tip, root_P_box_point)
        self.tip_P_goal_point.header.frame_id = self.tip
        # self.tip_P_goal_point.point.z = self.tip_P_goal_point.point.z - grasping_difference
        # TODO Calculate tip offset correctly. HSR will now push objects a little bit

        # bar_center
        self.bar_center = self.tip_P_goal_point

        # bar_length
        self.bar_length = 0.01

        self.tip_horizontal_axis = Vector3Stamped()
        self.tip_horizontal_axis.header.frame_id = self.tip_str
        self.tip_horizontal_axis.vector.x = 1

        # bar_axis
        hardcoded_y = Vector3(x=1, y=0, z=0)
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = "base_link"

        #self.bar_axis.vector = self.set_grasp_axis(self.object_size, maximum=True)
        self.bar_axis.vector = hardcoded_y


        # Align Planes
        # object axis horizontal/vertical
        # TODO replace with object orientation
        self.obj_front_axis = Vector3Stamped()
        self.obj_front_axis.header.frame_id = 'base_link'
        self.obj_front_axis.vector.z = -1

        # align z tip axis with object axis
        self.tip_front_axis = Vector3Stamped()
        self.tip_front_axis.header.frame_id = self.tip_str
        self.tip_front_axis.vector.z = 1

    def make_constraints(self):
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.obj_front_axis,
                                                 tip_normal=self.tip_front_axis))

        self.add_constraints_of_goal(GraspBar(root_link=self.root_str,
                                              tip_link=self.tip_str,
                                              tip_grasp_axis=self.tip_horizontal_axis,
                                              bar_center=self.bar_center,
                                              bar_axis=self.bar_axis,
                                              bar_length=self.bar_length))

    def __str__(self) -> str:
        return super().__str__()


class GraspFrontal(Goal):
    def __init__(self,
                 object_name: Optional[str] = None,
                 object_pose: Optional[PoseStamped] = None,
                 object_size: Optional[Vector3] = None,
                 object_geometry: Optional[LinkGeometry] = None,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link',
                 offset: Optional[float] = 0.01):
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
        self.root = self.world.get_link_name(root_link, None)
        loginfo(f'root_link: {self.root}')
        self.root_str = str(self.root)

        # tip link
        self.tip = self.world.get_link_name(tip_link, None)
        loginfo(f'tip_link: {self.tip}')
        self.tip_str = str(self.tip)

        # Grasp slightly below the center of the object
        object_pose.pose.position.z = object_pose.pose.position.z - 0.02
        self.object_pose = object_pose

        # Frame/grasp difference
        frame_difference = 0.05
        object_axis_size = 2 * frame_difference

        if isinstance(object_geometry, BoxGeometry):
            self.object_size = [object_geometry.width, object_geometry.depth, object_geometry.height]

            grasp_axis = self.set_grasp_axis(self.object_size, maximum=False)

            if grasp_axis.x == 1:
                object_axis_size = object_geometry.depth / 2

            elif grasp_axis.y == 1:
                object_axis_size = object_geometry.width / 2
        else:
            self.object_size = [object_size.x, object_size.y, object_size.z]

        grasping_difference = (object_axis_size + offset) - frame_difference
        grasping_difference = 0.082 # Offset for shelf handle
        grasping_difference = 0.078

        grasping_difference = max(0.001, 0.098 - (object_size.y/2))
        print(object_size)
        print(grasping_difference)


        root_P_box_point = PointStamped()
        root_P_box_point.header.frame_id = self.root_str
        root_P_box_point.point = self.object_pose.pose.position

        # Root -> Base link for hand_palm_link offset
        base_P_goal_point = self.transform_msg('base_link', root_P_box_point)
        base_P_goal_point.header.frame_id = 'base_link'
        base_P_goal_point.point.x = base_P_goal_point.point.x - grasping_difference


        # root -> tip tranfsormation
        self.tip_P_goal_point = self.transform_msg(self.tip, base_P_goal_point)
        self.tip_P_goal_point.header.frame_id = self.tip
        #self.tip_P_goal_point.point.z = self.tip_P_goal_point.point.z - grasping_difference
        # TODO Calculate tip offset correctly. HSR will now push objects a little bit

        # tip_axis
        self.tip_horizontal_axis = Vector3Stamped()
        self.tip_horizontal_axis.header.frame_id = self.tip_str
        self.tip_horizontal_axis.vector.x = 1

        # bar_center
        self.bar_center = self.tip_P_goal_point

        # bar_axis
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = self.root_str
        self.bar_axis.vector = self.set_grasp_axis(self.object_size, maximum=True)

        # bar length
        tolerance = 0.5
        self.bar_length = 0.00001  # max(self.obj_size) * tolerance

        # Align Planes
        # object axis horizontal/vertical
        # TODO replace with object orientation
        self.obj_front_axis = Vector3Stamped()
        self.obj_front_axis.header.frame_id = 'base_link'
        self.obj_front_axis.vector.x = 1

        # align z tip axis with object axis
        self.tip_front_axis = Vector3Stamped()
        self.tip_front_axis.header.frame_id = self.tip_str
        self.tip_front_axis.vector.z = 1

    def make_constraints(self):

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_str,
                                                 tip_link=self.tip_str,
                                                 goal_normal=self.obj_front_axis,
                                                 tip_normal=self.tip_front_axis))

        self.add_constraints_of_goal(GraspBar(root_link=self.root_str,
                                              tip_link=self.tip_str,
                                              tip_grasp_axis=self.tip_horizontal_axis,
                                              bar_center=self.bar_center,
                                              bar_axis=self.bar_axis,
                                              bar_length=self.bar_length))

    def __str__(self) -> str:
        return super().__str__()

    def set_grasp_axis(self, axes: List[float],
                       maximum: Optional[bool] = False):
        values = axes.copy()
        values.sort(reverse=maximum)

        index_sorted_values = []
        for e in values:
            index_sorted_values.append(axes.index(e))

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
                 tip_link: str = 'hand_palm_link',
                 weight=WEIGHT_ABOVE_CA):
        super().__init__()

        self.root_link = root_link
        self.tip_link = tip_link
        self.lifting = lifting
        if isinstance(weight, float):
            self.weight = weight
        else:
            self.weight = weight()

        print(f'LiftObject: {self.weight}')
        print(type(self.weight))

    def make_constraints(self):
        # Lifting
        goal_position = PointStamped()
        goal_position.header.frame_id = self.tip_link
        goal_position.point.x += self.lifting

        # Align Horizontal
        map_z = Vector3Stamped()
        map_z.header.frame_id = self.root_link
        map_z.vector.z = 1

        tip_horizontal = Vector3Stamped()
        tip_horizontal.header.frame_id = self.tip_link
        tip_horizontal.vector.x = 1

        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_link,
                                                       tip_link=self.tip_link,
                                                       goal_point=goal_position,
                                                       weight=self.weight))

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_link,
                                                 tip_link=self.tip_link,
                                                 goal_normal=map_z,
                                                 tip_normal=tip_horizontal,
                                                 weight=self.weight))

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

        self.root_link = root_link
        self.tip_link = tip_link
        self.distance = distance

        if isinstance(weight, float):
            self.weight = weight
        else:
            self.weight = weight()

        print(f'Retract: {self.weight}')
        print(type(self.weight))

    def make_constraints(self):
        goal_point = PointStamped()
        goal_point.header.frame_id = self.tip_link
        goal_point.point.x -= self.distance
        self.add_constraints_of_goal(CartesianPositionStraight(root_link=self.root_link,
                                                               tip_link=self.tip_link,
                                                               goal_point=goal_point,
                                                               weight=self.weight))

        # Algin Hand Horizontal
        map_z = Vector3Stamped()
        map_z.header.frame_id = self.root_link
        map_z.vector.z = 1

        tip_horizontal = Vector3Stamped()
        tip_horizontal.header.frame_id = 'hand_palm_link'
        tip_horizontal.vector.x = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_link,
                                                 tip_link=self.tip_link,
                                                 goal_normal=map_z,
                                                 tip_normal=tip_horizontal,
                                                 weight=self.weight))

    def __str__(self) -> str:
        return super().__str__()


class PreparePlacing(Goal):
    def __init__(self,
                 target_pose: PoseStamped,
                 object_height: float,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link'):
        super().__init__()

        self.root_link = self.world.get_link_name(root_link)
        self.tip_link = self.world.get_link_name(tip_link)

        # Pointing
        goal_point = PointStamped()
        goal_point.header.frame_id = root_link
        goal_point.point.x = target_pose.pose.position.x
        goal_point.point.y = target_pose.pose.position.y
        goal_point.point.z = target_pose.pose.position.z

        root_P_goal_point = self.transform_msg(tip_link, goal_point)

        # root_P_goal_point.point.x = 0
        root_P_goal_point.point.x += (object_height / 2)
        root_P_goal_point.point.y = 0
        root_P_goal_point.point.z = 0

        # print(root_P_goal_point)
        '''
        self.add_constraints_of_goal(Pointing(root_link=root_link,
                                              tip_link=tip_link,
                                              goal_point=goal_point))
        '''

        # Align Planes
        # object axis horizontal/vertical
        bar_axis_b = Vector3Stamped()
        bar_axis_b.header.frame_id = 'base_link'
        bar_axis_b.vector.y = 1

        # align z tip axis with object axis
        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = tip_link
        tip_grasp_axis_b.vector.z = 1

        '''
        self.add_constraints_of_goal(AlignPlanes(root_link=root_link,
                                                 tip_link=tip_link,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))
        '''

        # Algin Horizontal
        map_z = Vector3Stamped()
        map_z.header.frame_id = root_link
        map_z.vector.z = 1

        tip_horizontal = Vector3Stamped()
        tip_horizontal.header.frame_id = tip_link
        tip_horizontal.vector.x = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=root_link,
                                                 tip_link=tip_link,
                                                 goal_normal=map_z,
                                                 tip_normal=tip_horizontal))

        # Align height
        self.add_constraints_of_goal(CartesianPositionStraight(root_link=root_link,
                                                               tip_link=tip_link,
                                                               goal_point=root_P_goal_point))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class PlaceObject(Goal):
    def __init__(self,
                 object_name: str,
                 target_pose: PoseStamped,
                 object_height: float,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link'):
        super().__init__()

        self.root_link = root_link
        self.tip_link = tip_link
        giskard_link_name = str(self.world.get_link_name(self.tip_link))

        target_pose.pose.position.z = target_pose.pose.position.z + (object_height / 2)

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = "base_link"
        bar_axis.vector.x = 1

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = giskard_link_name
        tip_grasp_axis.vector.z = 1

        bar_axis_b = Vector3Stamped()
        bar_axis_b.header.frame_id = self.root_link
        bar_axis_b.vector.z = 1

        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = giskard_link_name
        tip_grasp_axis_b.vector.x = 1

        # align towards object
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_link,
                                                 tip_link=self.tip_link,
                                                 goal_normal=bar_axis,
                                                 tip_normal=tip_grasp_axis))

        goal_point = PointStamped()
        goal_point.header.frame_id = self.root_link
        goal_point.point.x = target_pose.pose.position.x
        goal_point.point.y = target_pose.pose.position.y
        goal_point.point.z = target_pose.pose.position.z

        '''
        self.add_constraints_of_goal(Pointing(tip_link=tip_link,
                                              goal_point=goal_point,
                                              root_link=root_link))
        '''
        # align horizontal
        self.add_constraints_of_goal(AlignPlanes(root_link=self.root_link,
                                                 tip_link=self.tip_link,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))
        # q = quaternion_from_matrix()
        # QuaternionStamped(*q)
        # CartesianOrientation

        # Move to Position
        self.add_constraints_of_goal(CartesianPosition(root_link=self.root_link,
                                                       tip_link=self.tip_link,
                                                       goal_point=goal_point))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()
