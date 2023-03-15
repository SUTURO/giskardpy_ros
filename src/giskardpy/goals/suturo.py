from typing import Optional, Tuple, List

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped, Point, Pose

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPose, CartesianPositionStraight, CartesianPosition
from giskardpy.goals.goal import Goal
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPosition, JointPositionList
from giskardpy.goals.pointing import Pointing
from giskardpy.model.utils import make_world_body_box, make_world_body_cylinder, make_world_body_sphere
from giskardpy.utils.logging import loginfo
from suturo_manipulation.gripper import Gripper

class SetBasePosition(Goal):
    def __init__(self):
        super().__init__()

        goal_new_state = {'arm_roll_joint': 1.6}
        self.add_constraints_of_goal(JointPositionList(goal_state=goal_new_state))

        loginfo(f'Moved hand out of sight')

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


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


class AddObjectToWorld(Goal):
    def __init__(self,
                 object_name: str,
                 object_pose: PoseStamped,):

        super().__init__()

        ### Will be removed with knowledge synchronization ###
        object_type = 'box'
        object_size = [0.04, 0.1, 0.1]

        # Create object body
        #if object_type == 'box':
            # object_size = [0.04, 0.1, 0.2]
        obj_body = make_world_body_box(object_size[0], object_size[1], object_size[2])
        print("added box")
        '''
        elif object_type == 'cylinder':
            object_height = 0.259
            radius = 0.0395
            obj_body = make_world_body_cylinder(object_height, radius)
            print('added cylinder')

        elif object_type == 'sphere':
            radius = 0.0395
            obj_body = make_world_body_sphere(radius)
            print('added sphere')

        else:
            obj_body = Pose()
            print("No object body to add")
        '''

        # Set object Pose
        obj_pose = Pose()
        obj_pose.position.x = object_pose.pose.position.x
        obj_pose.position.y = object_pose.pose.position.y
        obj_pose.position.z = object_pose.pose.position.z
        obj_pose.orientation.x = object_pose.pose.orientation.x
        obj_pose.orientation.y = object_pose.pose.orientation.y
        obj_pose.orientation.z = object_pose.pose.orientation.z

        # Parent link
        obj_parent_link = ''
        obj_parent_link_group = ''
        parent_link = self.world.get_link_name(obj_parent_link, obj_parent_link_group)

        # Add Object to giskard # FIXME Currently crashing grasp action
        self.world.add_world_body(object_name, obj_body, obj_pose, parent_link)


    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class GraspObject(Goal):
    def __init__(self,
                 object_name: str,
                 object_pose: PoseStamped,
                 # box_size: Vector3,
                 object_size: Optional[List[float]] = [0.04, 0.1, 0.2],
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link'
                 ):
        """
        Move to a given position where a box can be grasped.

        :param object_name: name of the object
        :param object_pose: center position of the grasped object
        :param object_size: box size as Vector3 (x, y, z)
        :param tip_link: name of the tip link of the kin chain

        """
        super().__init__()

        # giskard_link_name = self.world.get_link_name(tip_link)
        # root_link = self.world.root_link_name
        # map_box_pose = self.transform_msg('map', box_pose)

        def set_grasp_axis(axes: List[float],
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

        object_size = [0.04, 0.1, 0.2]

        # Frame/grasp difference
        grasping_difference = 0.07

        box_point = PointStamped()
        box_point.header.frame_id = root_link
        box_point.point.x = object_pose.pose.position.x
        box_point.point.y = object_pose.pose.position.y - grasping_difference
        box_point.point.z = object_pose.pose.position.z

        # root link
        self.root = self.world.get_link_name(root_link, None)
        self.tip = self.world.get_link_name(tip_link, None)
        self.root_P_goal_point = self.transform_msg(self.root, box_point)

        # tip link
        giskard_link_name = str(self.world.get_link_name(tip_link))
        loginfo('giskard_link_name: {}'.format(giskard_link_name))

        # box_size_array = [box_size.x, box_size.y, box_size.z]
        box_size_array = [object_size[0], object_size[1], object_size[2]]

        # tip_axis
        tip_grasp_a = Vector3Stamped()
        tip_grasp_a.header.frame_id = giskard_link_name
        tip_grasp_a.vector.x = 1

        object_pose.pose.position.y = object_pose.pose.position.y - grasping_difference

        # bar_center
        bar_c = PointStamped()
        bar_c.point = object_pose.pose.position

        # bar_axis
        bar_a = Vector3Stamped()
        bar_a.header.frame_id = root_link
        bar_a.vector = set_grasp_axis(box_size_array, maximum=True)

        # bar length
        tolerance = 0.5
        bar_l = max(box_size_array) * tolerance

        # Align Planes
        # object axis horizontal/vertical
        bar_axis_b = Vector3Stamped()
        bar_axis_b.header.frame_id = 'base_link'
        bar_axis_b.vector.x = 1

        # align z tip axis with object axis
        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = giskard_link_name
        tip_grasp_axis_b.vector.z = 1

        self.add_constraints_of_goal(Pointing(root_link=root_link,
                                              tip_link=giskard_link_name,
                                              goal_point=box_point))

        self.add_constraints_of_goal(AlignPlanes(root_link=root_link,
                                                 tip_link=giskard_link_name,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))

        self.add_constraints_of_goal(GraspBar(root_link=root_link,
                                              tip_link=giskard_link_name,
                                              tip_grasp_axis=tip_grasp_a,
                                              bar_center=bar_c,
                                              bar_axis=bar_a,
                                              bar_length=bar_l))



    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class PlaceObject(Goal):
    def __init__(self,
                 object_name: str,
                 target_pose: PoseStamped,
                 object_height: Optional[float] = 0.01,
                 root_link: Optional[str] = 'map',
                 tip_link: Optional[str] = 'hand_palm_link'):
        super().__init__()

        object_height = 0.28

        root_l = root_link
        tip_l = tip_link
        giskard_link_name = str(self.world.get_link_name(tip_l))

        target_pose.pose.position.z = target_pose.pose.position.z + (object_height / 2)

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = "base_link"
        bar_axis.vector.x = 1

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = giskard_link_name
        tip_grasp_axis.vector.z = 1

        bar_axis_b = Vector3Stamped()
        bar_axis_b.header.frame_id = root_l
        bar_axis_b.vector.z = 1

        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = giskard_link_name
        tip_grasp_axis_b.vector.x = 1

        # align towards object
        self.add_constraints_of_goal(AlignPlanes(root_link=root_l,
                                                 tip_link=giskard_link_name,
                                                 goal_normal=bar_axis,
                                                 tip_normal=tip_grasp_axis))

        goal_point = PointStamped()
        goal_point.header.frame_id = root_l
        goal_point.point.x = target_pose.pose.position.x
        goal_point.point.y = target_pose.pose.position.y
        goal_point.point.z = target_pose.pose.position.z

        '''
        self.add_constraints_of_goal(Pointing(tip_link=tip_link,
                                              goal_point=goal_point,
                                              root_link=root_link))
        '''
        # align horizontal
        self.add_constraints_of_goal(AlignPlanes(root_link=root_l,
                                                 tip_link=giskard_link_name,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))

        # Move to Position
        self.add_constraints_of_goal(CartesianPosition(root_link=root_l,
                                                       tip_link=giskard_link_name,
                                                       goal_point=goal_point))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class LiftObject(Goal):
    def __init__(self,
                 object_name: str,
                 lifting: Optional[float] = 0.02,
                 tip_link: Optional[str] = 'hand_palm_link'):
        super().__init__()

        root_name = 'map'

        # Lifting
        goal_position = PointStamped()
        goal_position.header.frame_id = tip_link
        goal_position.point.x += lifting

        # Algin Horizontal
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
                 root: Optional[str] = 'map',
                 tip: Optional[str] = 'base_link'):
        super().__init__()

        root_l = root
        tip_l = tip

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

        # Neutral Position
        joint_list = {
            u'head_pan_joint': 0.0,
            u'head_tilt_joint': 0.0,
            u'arm_lift_joint': 0.0,
            u'arm_flex_joint': 0.0,
            u'arm_roll_joint': 1.4,
            u'wrist_flex_joint': -1.5,
            u'wrist_roll_joint': 0.14
        }

        # self.add_constraints_of_goal(JointPositionList(goal_state=joint_list))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()
