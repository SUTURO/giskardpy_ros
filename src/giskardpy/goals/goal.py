from __future__ import annotations

import abc
from abc import ABC
from typing import Dict, List, Union, Optional

import controller_manager_msgs
# import giskardpy.identifier as identifier
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
from giskardpy.monitors.monitors import Monitor
from giskardpy.symbol_manager import symbol_manager
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
                 end_condition: cas.Expression = cas.FalseSymbol):
        """
        This is where you specify goal parameters and save them as self attributes.
        """
        self.tasks = []
        self.name = name

        self.standard_forward, self.standard_left, self.standard_up = None, None, None
        self.gripper_forward, self.gripper_left, self.gripper_up = None, None, None
        self.base_forward, self.base_left, self.base_up = None, None, None
        self.gripper_tool_frame = None

        if god_map.world.robot_name == 'hsrb':
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

    def connect_start_condition_to_all_tasks(self, condition: cas.Expression) -> None:
        for task in self.tasks:
            if cas.is_true(task.start_condition):
                task.start_condition = condition
            else:
                task.start_condition = cas.logic_and(task.start_condition, condition)

    def connect_hold_condition_to_all_tasks(self, condition: cas.Expression) -> None:
        for task in self.tasks:
            if cas.is_false(task.hold_condition):
                task.hold_condition = condition
            else:
                task.hold_condition = cas.logic_or(task.hold_condition, condition)

    def connect_end_condition_to_all_tasks(self, condition: cas.Expression) -> None:
        for task in self.tasks:
            if cas.is_false(task.end_condition):
                task.end_condition = condition
            else:
                task.end_condition = cas.logic_and(task.end_condition, condition)

    def connect_monitors_to_all_tasks(self,
                                      start_condition: cas.Expression,
                                      hold_condition: cas.Expression,
                                      end_condition: cas.Expression):
        self.connect_start_condition_to_all_tasks(start_condition)
        self.connect_hold_condition_to_all_tasks(hold_condition)
        self.connect_end_condition_to_all_tasks(end_condition)

    @property
    def ref_str(self) -> str:
        """
        A string referring to self on the god_map. Used with symbol manager.
        """
        return f'god_map.motion_goal_manager.motion_goals[\'{str(self)}\']'

    def __add__(self, other: str) -> str:
        if isinstance(other, str):
            return self.ref_str + other
        raise NotImplementedError('Goal can only be added with a string.')

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

    def add_monitor(self, monitor: Monitor) -> None:
        if isinstance(monitor, ExpressionMonitor):
            god_map.monitor_manager.add_expression_monitor(monitor)
        else:
            god_map.monitor_manager.add_payload_monitor(monitor)

