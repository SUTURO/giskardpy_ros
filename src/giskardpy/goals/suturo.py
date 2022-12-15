from typing import Optional

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Vector3Stamped

from giskardpy.goals.align_planes import AlignPlanes
from giskardpy.goals.cartesian_goals import CartesianPose, CartesianPositionStraight, CartesianPosition
from giskardpy.goals.goal import Goal
from giskardpy.goals.grasp_bar import GraspBar
from giskardpy.goals.joint_goals import JointPosition, JointPositionList
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

class PrepareGraspBox(Goal):
    def __init__(self,
                 box_pose: PoseStamped,
                 tip_link: Optional[str] = 'hand_palm_link',
                 box_z_length: Optional[float] = 0.001,
                 box_x_length: Optional[float] = None,
                 box_y_length: Optional[float] = None,
                 grasp_type: Optional[bool] = True,
                 grasp_vertical: Optional[bool] = False):

        """
        Move to a given position where a box can be grasped.

        :param box_pose: center position of the grasped object
        :param tip_link: name of the tip link of the kin chain
        :param box_z_length: length of the box along the z-axis
        :param box_x_length: length of the box along the x-axis
        :param box_y_length: length of the box along the y-axis
        :param grasp_type: parameter to decide if the mueslibox is grasped
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
        if grasp_type:
            tip_grasp_a.vector.x = 1
        else:
            tip_grasp_a.vector.y = 1

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

        if grasp_type:
            tip_grasp_axis_b.vector.z = 1
        else:
            tip_grasp_axis_b.vector.z = -1

        bar_axis_b = Vector3Stamped()
        bar_axis_b.vector.y = 1

        self.add_constraints_of_goal(AlignPlanes(root_link=root_l,
                                                 tip_link=giskard_link_name,
                                                 goal_normal=bar_axis_b,
                                                 tip_normal=tip_grasp_axis_b))

        # align hand with z
        self.add_constraints_of_goal(GraspBar(root_link=root_l,
                                              tip_link=giskard_link_name,
                                              tip_grasp_axis=tip_grasp_a,
                                              bar_center=bar_c,
                                              bar_axis=bar_a,
                                              bar_length=bar_l))

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
