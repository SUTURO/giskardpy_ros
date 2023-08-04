import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from py_trees import Status
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from giskardpy import identifier
from giskardpy.tree.behaviors.plugin import GiskardBehavior


# Debugging
# import pydevd_pycharm
# pydevd_pycharm.settrace('localhost', port=1234, stdoutToServer=True, stderrToServer=True, suspend=False)


class SuturoGripperHandler(GiskardBehavior):
    """
    Handles the movement of the gripper.
    """

    @profile
    def __init__(self, name):
        super().__init__(name)

        self._gripper_controller = None
        self._gripper_apply_force_client = None
        self.name = name
        apply_force_action_server = '/hsrb/gripper_controller/grasp',
        follow_joint_trajectory_server = '/hsrb/gripper_controller/follow_joint_trajectory'

        self.apply_force_action_server = apply_force_action_server
        self.follow_joint_trajectory_server = follow_joint_trajectory_server


    @profile
    def setup(self, timeout):
        self._gripper_apply_force_client = actionlib.SimpleActionClient('/hsrb/gripper_controller/grasp',
                                                                        GripperApplyEffortAction)
        self._gripper_controller = actionlib.SimpleActionClient(self.follow_joint_trajectory_server,
                                                                FollowJointTrajectoryAction)
        self._gripper_apply_force_client.wait_for_server()

        self._gripper_controller = actionlib.SimpleActionClient('/hsrb/gripper_controller/follow_joint_trajectory',
                                                                FollowJointTrajectoryAction)
        self._gripper_controller.wait_for_server()

        self.god_map.set_data(identifier=identifier.gripper_controller, value=self.close_gripper_force)
        self.god_map.set_data(identifier=identifier.gripper_trajectory, value=self.set_gripper_joint_position)
        return True

    def close_gripper_force(self, force=0.8):
        """
        Closes the gripper with the given force.
        :param force: force to grasp with should be between 0.2 and 0.8 (N)
        :return: applied effort
        """
        rospy.loginfo("Closing gripper with force: {}".format(force))
        f = force # max(min(0.8, force), 0.2)
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

    @profile
    def update(self):
        return Status.SUCCESS

#    def terminate(self, new_status):
#        self.tree.remove_node(self.name)
