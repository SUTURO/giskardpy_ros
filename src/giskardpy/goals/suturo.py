from typing import Tuple, Optional

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPose, CartesianPositionStraight, CartesianPosition
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPosition, JointPositionList
from giskardpy.goals.open_close import Open
from giskardpy.utils.logging import loginfo


class MoveHandOutOfSight(Goal):
    def __init__(self):
        super().__init__()

        '''
        root_link = str(self.world.root_link_name)
        root_group = None
        tip_group = None

        max_linear_velocity = None
        reference_linear_velocity = None
        weight = WEIGHT_ABOVE_CA
        
        self.add_constraints_of_goal(CartesianPositionStraight(root_link=root_link,
                                                               root_group=root_group,
                                                                tip_link=tip_link,
                                                               tip_group=tip_group,
                                                               goal_point=goal_point,
                                                               max_velocity=max_linear_velocity,
                                                               reference_velocity=reference_linear_velocity,
                                                                weight=weight))
        General movement goal
        '''

        goal_new_state = {'arm_roll_joint': 1.6}
        self.add_constraints_of_goal(JointPositionList(goal_state=goal_new_state))
        # logging: loginfo(f'Moved hand out of sight')

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class Test(Goal):
    def __init__(self,
                 tip_link: str,
                 environment_link: str,
                 tip_group: Optional[str] = None,
                 environment_group: Optional[str] = None,
                 goal_joint_state: Optional[float] = None,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Open a container in an environment.
        Only works with the environment was added as urdf.
        Assumes that a handle has already been grasped.
        Can only handle containers with 1 dof, e.g. drawers or doors.
        :param tip_link: end effector that is grasping the handle
        :param environment_link: name of the handle that was grasped
        :param tip_group: if tip_link is not unique, search in this group for matches
        :param environment_group: if environment_link is not unique, search in this group for matches
        :param goal_joint_state: goal state for the container. default is maximum joint state.
        :param weight:
        """
        super().__init__()
        self.weight = weight
        self.tip_link = self.world.get_link_name(tip_link, tip_group)
        self.handle_link = self.world.get_link_name(environment_link, environment_group)
        self.joint_name = self.world.get_movable_parent_joint(self.handle_link)
        self.joint_group = self.world.get_group_of_joint(self.joint_name)
        self.handle_T_tip = self.world.compute_fk_pose(self.handle_link, self.tip_link)

        _, max_position = self.world.get_joint_position_limits(self.joint_name)
        if goal_joint_state is None:
            goal_joint_state = max_position
        else:
            goal_joint_state = min(max_position, goal_joint_state)

        self.add_constraints_of_goal(CartesianPose(root_link=environment_link,
                                                   root_group=environment_group,
                                                   tip_link=tip_link,
                                                   tip_group=tip_group,
                                                   goal_pose=self.handle_T_tip,
                                                   weight=self.weight * 100))
        self.add_constraints_of_goal(JointPosition(joint_name=self.joint_name.short_name,
                                                   group_name=self.joint_group.name,
                                                   goal=goal_joint_state,
                                                   weight=weight))


class PrepareGraspBox(Goal):
    def __init__(self,
                 box_pose: PoseStamped,
                 tip_link: str,
                 box_z_length: float,
                 box_x_length: Optional[float] = None,
                 box_y_length: Optional[float] = None,
                 mueslibox: Optional[bool] = False,
                 grasp_vertical: Optional[bool] = False):

        """
        Move to a given position where a box can be grasped.

        :param box_pose: center position of the grasped object
        :param tip_link: name of the tip link of the kin chain
        :param box_z_length: length of the box along the z-axis
        :param box_x_length: length of the box along the x-axis
        :param box_y_length: length of the box along the y-axis
        :param mueslibox: parameter to decide if the mueslibox iis grasped
        :param grasp_vertical: parameter to align the gripper vertical

        """
        super().__init__()
        # giskard_link_name = self.world.get_link_name(tip_link)
        # root_link = self.world.root_link_name
        # map_box_pose = self.transform_msg('map', box_pose)

        # root link
        root_l = 'map'

        # tip link
        giskard_link_name = str(self.world.get_link_name(tip_link))
        loginfo('giskard_link_name: {}'.format(giskard_link_name))

        # tip_axis
        tip_grasp_a = Vector3Stamped()
        tip_grasp_a.header.frame_id = giskard_link_name
        if grasp_vertical:
            tip_grasp_a.vector.y = 1
        else:
            tip_grasp_a.vector.x = 1

        # bar_center
        bar_c = PointStamped()
        bar_c.point = box_pose.pose.position

        # bar_axis
        bar_a = Vector3Stamped()
        bar_a.vector.z = 1

        bar_l = box_z_length

        # align with axis
        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = giskard_link_name

        if mueslibox:
            tip_grasp_axis_b.vector.z = 1
        else:
            tip_grasp_axis_b.vector.z = -1

        bar_axis_b = Vector3Stamped()
        bar_axis_b.vector.y = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=root_l,
                                                 tip_link=giskard_link_name,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))

        # '''
        # align hand with z
        self.add_constraints_of_goal(GraspBar(root_link=root_l,
                                              tip_link=giskard_link_name,
                                              tip_grasp_axis=tip_grasp_a,
                                              bar_center=bar_c,
                                              bar_axis=bar_a,
                                              bar_length=bar_l))
        # '''

        # TODO raise arm

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class MoveDrawer(Goal):
    def __init__(self,
                 knob_pose: PoseStamped,
                 direction: Vector3,
                 distance: float):

        """
        Move drawer in a given direction.
        The drawer knob has to be grasped first, f.e. with PrepareGraspBox.
        :param knob_pose: current position of the knob
        :param direction: direction vector in which the drawer should move
        :param distance: distance the drawer should move
        """

        super().__init__()

        root_l = 'map'
        giskard_link_name = str(self.world.get_link_name('hand_palm_link'))

        new_x = (direction.x * distance) + knob_pose.pose.position.x
        new_y = (direction.y * distance) + knob_pose.pose.position.y
        new_z = (direction.z * distance) + knob_pose.pose.position.z
        calculated_position = Vector3(new_x, new_y, new_z)

        goal_position = PoseStamped()
        goal_position.header.frame_id = root_l
        goal_position.pose.position = calculated_position
        loginfo(goal_position)

        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = giskard_link_name
        tip_grasp_axis_b.vector.z = -1

        bar_axis_b = Vector3Stamped()
        bar_axis_b.vector.y = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=root_l,
                                                 tip_link=giskard_link_name,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))

        self.add_constraints_of_goal(CartesianPose(root_link=root_l,
                                                   tip_link=giskard_link_name,
                                                   goal_pose=goal_position))

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()
