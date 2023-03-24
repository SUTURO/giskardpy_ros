from typing import Optional, List

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, QuaternionStamped
from tf.transformations import quaternion_from_matrix

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPositionStraight, CartesianPosition, CartesianOrientation
from giskardpy.goals.goal import Goal
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPositionList
from giskardpy.goals.pointing import Pointing
from giskardpy.model.links import BoxGeometry
from giskardpy.utils.logging import loginfo
from suturo_manipulation.gripper import Gripper


class MoveGripper(Goal):
    def __init__(self, open_gripper=True):
        """
        Open / CLose Gripper.
        Current implementation is not final and will be replaced with a follow joint trajectory connection.

        :param open_gripper: True to open gripper; False to close gripper.
        """

        super().__init__()
        g = Gripper(apply_force_action_server='/hsrb/gripper_controller/apply_force',
                    follow_joint_trajectory_server='/hsrb/gripper_controller/follow_joint_trajectory')

        if open_gripper:
            g.set_gripper_joint_position(1)
        else:
            g.close_gripper_force(1)

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class GraspObject(Goal):
    def __init__(self,
                 object_name: str,
                 object_pose: PoseStamped,
                 object_size: Vector3,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link'
                 ):
        """
        Move to a given position where a box can be grasped.

        :param object_name: name of the object
        :param object_pose: center position of the grasped object
        :param object_size: box size as Vector3 (x, y, z)
        :param root_link: name of the root link of the kin chain
        :param tip_link: name of the tip link of the kin chain

        """
        super().__init__()

        obj_size = [object_size.x, object_size.y, object_size.z]
        # geometry: BoxGeometry = self.world.groups['box'].root_link.collisions[0] # how to get geometry of a link

        # Frame/grasp difference
        grasping_difference = 0.04

        root_P_box_point = PointStamped()
        root_P_box_point.header.frame_id = root_link
        root_P_box_point.point = object_pose.pose.position

        # root link
        self.root = self.world.get_link_name(root_link, None)
        # tip link
        self.tip = self.world.get_link_name(tip_link, None)

        # root -> tip tranfsormation
        self.tip_P_goal_point = self.transform_msg(self.tip, root_P_box_point)
        self.tip_P_goal_point.header.frame_id = self.tip
        # TODO Calculate tip offset correctly. HSR will now push objects a little bit
        # self.tip_P_goal_point.point.z = self.tip_P_goal_point.point.z - grasping_difference

        # tip link
        giskard_link_name = str(self.tip)
        loginfo(f'giskard_link_name: {self.tip}')

        # tip_axis
        self.tip_horizontal_axis = Vector3Stamped()
        self.tip_horizontal_axis.header.frame_id = giskard_link_name
        self.tip_horizontal_axis.vector.x = 1

        # bar_center
        self.bar_center = self.tip_P_goal_point

        # bar_axis
        self.bar_axis = Vector3Stamped()
        self.bar_axis.header.frame_id = root_link
        self.bar_axis.vector = self.set_grasp_axis(obj_size, maximum=True)

        # bar length
        tolerance = 0.5
        self.bar_length = max(obj_size) * tolerance

        # Align Planes
        # object axis horizontal/vertical
        self.obj_front_axis = Vector3Stamped()
        self.obj_front_axis.header.frame_id = 'base_link'
        self.obj_front_axis.vector.x = 1

        # align z tip axis with object axis
        self.tip_front_axis = Vector3Stamped()
        self.tip_front_axis.header.frame_id = giskard_link_name
        self.tip_front_axis.vector.z = 1

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

    def make_constraints(self):
        root_link = str(self.root)
        tip_link = str(self.tip)

        self.add_constraints_of_goal(AlignPlanes(root_link=root_link,
                                                 tip_link=tip_link,
                                                 goal_normal=self.obj_front_axis,
                                                 tip_normal=self.tip_front_axis))

        self.add_constraints_of_goal(GraspBar(root_link=root_link,
                                              tip_link=tip_link,
                                              tip_grasp_axis=self.tip_horizontal_axis,
                                              bar_center=self.bar_center,
                                              bar_axis=self.bar_axis,
                                              bar_length=self.bar_length))

    def __str__(self) -> str:
        return super().__str__()


class LiftObject(Goal):
    def __init__(self,
                 object_name: str,
                 lifting: float = 0.02,
                 tip_link: str = 'hand_palm_link'):
        super().__init__()

        root_name = 'map'

        # Lifting
        goal_position = PointStamped()
        goal_position.header.frame_id = tip_link
        goal_position.point.x += lifting

        # Align Horizontal
        map_z = Vector3Stamped()
        map_z.header.frame_id = root_name
        map_z.vector.z = 1

        tip_horizontal = Vector3Stamped()
        tip_horizontal.header.frame_id = tip_link
        tip_horizontal.vector.x = 1

        self.add_constraints_of_goal(CartesianPosition(root_link=root_name,
                                                       tip_link=tip_link,
                                                       goal_point=goal_position))

        self.add_constraints_of_goal(AlignPlanes(root_link=root_name,
                                                 tip_link=tip_link,
                                                 goal_normal=map_z,
                                                 tip_normal=tip_horizontal))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class Retracting(Goal):
    def __init__(self,
                 object_name: str,
                 distance: Optional[float] = 0.2,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'base_link'):
        super().__init__()

        root_l = root_link
        tip_l = tip_link

        goal_point = PointStamped()
        goal_point.header.frame_id = tip_l
        goal_point.point.x -= distance
        self.add_constraints_of_goal(CartesianPositionStraight(root_link=root_l,
                                                               tip_link=tip_l,
                                                               goal_point=goal_point))

        # Algin Horizontal
        map_z = Vector3Stamped()
        map_z.header.frame_id = root_l
        map_z.vector.z = 1

        tip_horizontal = Vector3Stamped()
        tip_horizontal.header.frame_id = 'hand_palm_link'
        tip_horizontal.vector.x = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=root_l,
                                                 tip_link=tip_l,
                                                 goal_normal=map_z,
                                                 tip_normal=tip_horizontal))

    def make_constraints(self):
        pass

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
