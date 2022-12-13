from typing import Tuple, Optional

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped

from giskardpy.goals.cartesian_goals import CartesianPose, CartesianPositionStraight
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
                 box_y_length: Optional[float] = None):
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
        tip_grasp_a.vector.x = 1
        #tip_grasp_a.vector.x = 1 # drawer

        # bar_center
        bar_c = PointStamped()
        bar_c.point = box_pose.point

        loginfo(box_pose.header.frame_id)

        loginfo(box_pose)
        loginfo(box_pose.point)

        # bar_axis
        bar_a = Vector3Stamped()
        bar_a.vector.z = 1
        #bar_a.vector.z = 1 # drawer

        bar_l = box_z_length

        # align with axis
        tip_grasp_axis_b = Vector3Stamped()
        tip_grasp_axis_b.header.frame_id = giskard_link_name
        tip_grasp_axis_b.vector.z = 1
        #tip_grasp_axis_b.vector.z = -1 # drawer

        bar_axis_b = Vector3Stamped()
        bar_axis_b.vector.y = 1
        #bar_axis_b.vector.y = 1 # drawer

        # TODO: open gripper

        # align hand with x
        self.add_constraints_of_goal(GraspBar(root_link=root_l,
                                              tip_link=giskard_link_name,
                                              tip_grasp_axis=tip_grasp_axis_b,
                                              bar_center=bar_c,
                                              bar_axis=bar_axis_b,
                                              bar_length=0.001))
        #'''
        # align hand with z
        self.add_constraints_of_goal(GraspBar(root_link=root_l,
                                              tip_link=giskard_link_name,
                                              tip_grasp_axis=tip_grasp_a,
                                              bar_center=bar_c,
                                              bar_axis=bar_a,
                                              bar_length=bar_l))
        #'''
        # plan and execute

        # TODO close gripper

        # plan and execute

        # TODO raise arm


    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()



class OpenDrawer(Goal):
    def __init__(self,
                 knob_pose: PointStamped,
                 direction: Vector3,
                 distance: float):

        # TODO Grasp drawer knob

        # TODO pull the knob in the given direction for the given distance
        pass

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()
