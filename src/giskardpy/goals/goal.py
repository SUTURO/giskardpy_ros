from __future__ import annotations

import abc
from abc import ABC
from typing import Dict, List, Union

import controller_manager_msgs
#import giskardpy.identifier as identifier
import rospy
import trajectory_msgs
from geometry_msgs.msg import Vector3

import giskardpy.casadi_wrapper as cas
from giskardpy.data_types import PrefixName, Derivatives
from giskardpy.exceptions import GiskardException
from giskardpy.exceptions import GoalInitalizationException
from giskardpy.god_map import god_map
from giskardpy.model.joints import OneDofJoint
from giskardpy.monitors.monitors import ExpressionMonitor
from giskardpy.tasks.task import Task
from giskardpy.tree.behaviors.suturo_monitor_force_sensor import MonitorForceSensor
# from giskardpy.tree.garden import success_is_failure
from giskardpy.utils import logging
from giskardpy.utils.utils import string_shortener


class Goal(ABC):
    tasks: List[Task]
    name: str

    @abc.abstractmethod
    def __init__(self,
                 name: str,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.TrueSymbol):
        """
        This is where you specify goal parameters and save them as self attributes.
        """
        self.tasks = []
        self.name = name

        self.standard_forward, self.standard_left, self.standard_up = None, None, None
        self.gripper_forward, self.gripper_left, self.gripper_up = None, None, None
        self.base_forward, self.base_left, self.base_up = None, None, None
        self.gripper_tool_frame = None

        if self.world.robot_name == 'hsrb':
            self.standard_forward = Vector3(x=1, y=0, z=0)
            self.standard_left = Vector3(x=0, y=1, z=0)
            self.standard_up = Vector3(x=0, y=0, z=1)

            self.gripper_forward = Vector3(x=0, y=0, z=1)
            self.gripper_left = Vector3(x=0, y=-1, z=0)
            self.gripper_up = Vector3(x=1, y=0, z=0)

            self.base_forward = Vector3(x=1, y=0, z=0)
            self.base_left = Vector3(x=0, y=1, z=0)
            self.base_up = Vector3(x=0, y=0, z=1)

            self.gripper_tool_frame = 'hand_gripper_tool_frame'

        elif self.world.robot_name == 'iai_donbot':
            self.standard_forward = Vector3(x=1, y=0, z=0)
            self.standard_left = Vector3(x=0, y=1, z=0)
            self.standard_up = Vector3(x=0, y=0, z=1)

            self.gripper_forward = Vector3(x=0, y=0, z=1)
            self.gripper_left = Vector3(x=1, y=0, z=0)
            self.gripper_up = Vector3(x=0, y=1, z=0)

            # FIXME: Look up the real values
            self.base_forward = Vector3(x=-1, y=0, z=0)
            self.base_left = Vector3(x=0, y=-1, z=0)
            self.base_up = Vector3(x=0, y=0, z=1)

            self.gripper_tool_frame = 'gripper_tool_frame'

    def formatted_name(self, quoted: bool = False) -> str:
        formatted_name = string_shortener(original_str=self.name,
                                          max_lines=4,
                                          max_line_length=25)
        if quoted:
            return '"' + formatted_name + '"'
        return formatted_name

    def clean_up(self):
        pass

    def is_done(self):
        return None

    def __str__(self) -> str:
        return self.name

    def __repr__(self) -> str:
        return self.name

    def has_tasks(self) -> bool:
        return len(self.tasks) > 0

    def get_joint_position_symbol(self, joint_name: PrefixName) -> Union[cas.Symbol, float]:
        """
        returns a symbol that refers to the given joint
        """
        if not god_map.world.has_joint(joint_name):
            raise KeyError(f'World doesn\'t have joint named: {joint_name}.')
        joint = god_map.world.joints[joint_name]
        if isinstance(joint, OneDofJoint):
            return joint.get_symbol(Derivatives.position)
        raise TypeError(f'get_joint_position_symbol is only supported for OneDofJoint, not {type(joint)}')

    def connect_start_condition_to_all_tasks(self, condition: cas.Expression):
        for task in self.tasks:
            task.start_condition = cas.logic_and(task.start_condition, condition)

    def connect_hold_condition_to_all_tasks(self, condition: cas.Expression):
        for task in self.tasks:
            task.hold_condition = cas.logic_or(task.hold_condition, condition)

    def connect_end_condition_to_all_tasks(self, condition: cas.Expression):
        for task in self.tasks:
            task.end_condition = cas.logic_and(task.end_condition, condition)

    def connect_monitors_to_all_tasks(self,
                                      start_condition: cas.Expression,
                                      hold_condition: cas.Expression,
                                      end_condition: cas.Expression):
        self.connect_start_condition_to_all_tasks(start_condition)
        self.connect_hold_condition_to_all_tasks(hold_condition)
        self.connect_end_condition_to_all_tasks(end_condition)

    def get_expr_velocity(self, expr: cas.Expression) -> cas.Expression:
        """
        Creates an expressions that computes the total derivative of expr
        """
        return cas.total_derivative(expr,
                                    self.joint_position_symbols,
                                    self.joint_velocity_symbols)

    @property
    def joint_position_symbols(self) -> List[Union[cas.Symbol, float]]:
        position_symbols = []
        for joint in god_map.world.controlled_joints:
            position_symbols.extend(god_map.world.joints[joint].free_variables)
        return [x.get_symbol(Derivatives.position) for x in position_symbols]

    @property
    def joint_velocity_symbols(self) -> List[Union[cas.Symbol, float]]:
        velocity_symbols = []
        for joint in god_map.world.controlled_joints:
            velocity_symbols.extend(god_map.world.joints[joint].free_variable_list)
        return [x.get_symbol(Derivatives.velocity) for x in velocity_symbols]

    @property
    def joint_acceleration_symbols(self) -> List[Union[cas.Symbol, float]]:
        acceleration_symbols = []
        for joint in god_map.world.controlled_joints:
            acceleration_symbols.extend(god_map.world.joints[joint].free_variables)
        return [x.get_symbol(Derivatives.acceleration) for x in acceleration_symbols]

    def _task_sanity_check(self):
        if not self.has_tasks():
            raise GoalInitalizationException(f'Goal {str(self)} has no tasks.')

    def add_constraints_of_goal(self, goal: Goal):
        for task in goal.tasks:
            if not [t for t in self.tasks if t.name == task.name]:
                self.tasks.append(task)
            else:
                raise GoalInitalizationException(f'Constraint with name {task.name} already exists.')

    def create_and_add_task(self, task_name: str = '') -> Task:
        task = Task(name=task_name, parent_goal_name=self.name)
        self.tasks.append(task)
        return task

    def add_monitor(self, monitor: ExpressionMonitor) -> None:
        god_map.monitor_manager.add_expression_monitor(monitor)


class NonMotionGoal(Goal):
    """
    Inherit from this goal, if the goal does not add any constraints.
    """
    pass


class ForceSensorGoal(Goal):
    """
    Inherit from this goal, if the goal should use the Force Sensor.
    """

    def __init__(self):
        super().__init__()

        robot_name = self.world.robot_name
        self.arm_trajectory_publisher = None

        if robot_name == 'hsrb':
            self.wrench_topic_name = '/hsrb/wrist_wrench/compensated'
            self.arm_topic_name = '/hsrb/arm_trajectory_controller/command'
            # self.controller_list_topic_name = '/hsrb/controller_manager/list_controllers'
            self.controller_list_topic_name = None
            self.directions = {'up': 'x',
                               'forward': 'z',
                               'side': 'y'}

        elif robot_name == 'iai_donbot':
            self.wrench_topic_name = '/kms40_driver/wrench'
            self.arm_topic_name = '/scaled_pos_joint_traj_controller/command'
            self.directions = {'up': 'y',
                               'forward': 'z',
                               'side': 'x'}
        else:
            logging.logerr(f'{robot_name} is not supported')
            raise GiskardException()

        self.upwards_force, self.forward_force, self.sideway_force = [value + '_force' for key, value in
                                                                      self.directions.items()]
        self.upwards_torque, self.forward_torque, self.sideway_torque = [value + '_torque' for key, value in
                                                                         self.directions.items()]

        conditions = self.goal_cancel_condition()
        tree = self.god_map.get_data(identifier=identifier.tree_manager)

        self.behaviour = MonitorForceSensor('monitor force', conditions, self.wrench_topic_name)

        if self.control_mode == self.control_mode.open_loop:
            self.connect_trajectory_publisher()

            tree.insert_node(self.behaviour, 'monitor execution', 2)
        else:
            tree.insert_node(self.behaviour, 'closed loop control', 12)

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()

    @abc.abstractmethod
    def goal_cancel_condition(self):
        pass

    def recovery_modifier(self) -> Dict:
        return {}

    def connect_trajectory_publisher(self):
        # initialize ROS publisher
        if self.arm_topic_name is not None:
            self.arm_trajectory_publisher = rospy.Publisher(self.arm_topic_name,
                                                            trajectory_msgs.msg.JointTrajectory, queue_size=10)

            # wait to establish connection between the controller
            while self.arm_trajectory_publisher.get_num_connections() == 0:
                logging.logwarn(f'connecting to {self.arm_topic_name}')
                rospy.sleep(0.1)

        # make sure the controller is running
        if self.controller_list_topic_name is not None:
            rospy.wait_for_service(self.controller_list_topic_name)
            list_controllers = (
                rospy.ServiceProxy(self.controller_list_topic_name,
                                   controller_manager_msgs.srv.ListControllers))

            running1, running2 = False, False
            while running1 is False or running2 is False:
                rospy.sleep(0.1)
                for c in list_controllers().controller:
                    if c.name == 'arm_trajectory_controller' and c.state == 'running':
                        running1 = True

    def recover(self, joint_modifier):

        arm_joint_names = ['arm_lift_joint', 'arm_flex_joint',
                           'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

        if any(x in joint_modifier for x in arm_joint_names):
            arm_joint_positions = []
            for joint_name in arm_joint_names:
                join_state_position = self.world.state.get(self.world.search_for_joint_name(joint_name)).position

                if joint_name in joint_modifier:
                    mod = joint_modifier.get(joint_name)
                else:
                    mod = 0.0

                arm_joint_positions.append(join_state_position + mod)

            # fill ROS message
            traj = trajectory_msgs.msg.JointTrajectory()
            traj.joint_names = arm_joint_names

            trajectory_point = trajectory_msgs.msg.JointTrajectoryPoint()
            trajectory_point.positions = arm_joint_positions
            trajectory_point.velocities = [0, 0, 0, 0, 0]
            trajectory_point.time_from_start = rospy.Duration(1)
            traj.points = [trajectory_point]

            # publish ROS message
            self.arm_trajectory_publisher.publish(traj)

    def clean_up(self):
        if self.control_mode == self.control_mode.open_loop:
            self.recover(self.recovery_modifier())

        try:
            self.behaviour.wrench_compensated_subscriber.unregister()
            logging.loginfo('Successfully unsubscribed force monitoring')
        except:
            logging.logwarn(f'Subscriber does not exist in {self.behaviour.name}')

        tree = self.tree_manager
        tree.remove_node(self.behaviour.name)
