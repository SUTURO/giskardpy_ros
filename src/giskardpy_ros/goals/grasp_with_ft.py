from typing import Optional

from giskardpy import casadi_wrapper as cas
from giskardpy.data_types.data_types import PrefixName
from giskardpy.data_types.exceptions import ObjectForceTorqueThresholdException
from giskardpy.data_types.suturo_types import ForceTorqueThresholds
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.motion_statechart.monitors.force_torque_monitor import PayloadForceTorque
from giskardpy.motion_statechart.monitors.monitors import CancelMotion
from giskardpy.motion_statechart.monitors.payload_monitors import Sleep
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition
from giskardpy.motion_statechart.tasks.grasp_bar import GraspBarOffset
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList

from giskardpy_ros.monitors.handle_offset_monitor import HandleOffsetCorrection


class GraspWithForceTorqueGoal(Goal):
    def __init__(self,
                 root_link: PrefixName,
                 tip_link: PrefixName,
                 handle_name: PrefixName,
                 tip_grasp_axis: cas.Vector3,
                 bar_axis: cas.Vector3,
                 tip_retract: cas.Point3,
                 handle_align_axis: cas.Vector3,
                 tip_align_axis: cas.Vector3,
                 grasp_axis_offset: cas.Vector3,
                 pre_grasp_axis_offset: cas.Vector3,
                 hinge_joint: PrefixName,
                 bar_length: float = 0.01,
                 timeout: float = 10,
                 ft_topic: str = '/filtered_raw/diff',
                 ft_grasp_ref_speed: float = 1,
                 camera_link: Optional[PrefixName] = None,
                 tip_push: Optional[cas.Point3] = None,
                 handle_correction_offset: Optional[cas.Point3] = None,
                 name: str = None):
        """
        Complex grasping motion using the ForceTorqueMonitor.

        :param root_link: root-link of the kinematic chain.
        :param tip_link: tip-link of the kinematic chain.
        :param handle_name: LinkName of the environment-link that is to be grasped.
        :param tip_grasp_axis: axis of tip-link that is to be aligned with bar axis.
        :param bar_axis: axis of the handle along which the handlebar is.
        :param tip_retract: distance the tip will retract after force-torque-threshold is reached.
        :param handle_align_axis: axis of the handle which will be aligned with the grasp_axis while grasping.
        :param tip_align_axis: axis of the tip_link along which is grasped. e.g. z-axis of the hsrb-gripper.
        :param grasp_axis_offset: offset for grasping, such that force-torque sensor can be triggered.
        :param pre_grasp_axis_offset: offset for pre-grasp-pose, from which the ft-grasping is started.
        :param hinge_joint: hinge joint to be locked in place while grasping.
        :param bar_length: length of the handlebar.
        :param timeout: duration after which the ft-grasping is cancelled.
        :param ft_topic: topic of the sensor-data for the ft-monitor.
        :param ft_grasp_ref_speed: reference-speed-modifier for the ft-grasping motion.
        :param name: Name of the Goal
        """
        if name is None:
            name = 'GraspWithForceTorqueGoal'
        super().__init__(name=name)

        pre_grasp_name = 'pre grasp'

        bar_center = cas.Point3()
        bar_center.reference_frame = handle_name

        self.reference_linear_velocity = 0.1 * ft_grasp_ref_speed
        self.reference_angular_velocity = 0.5 * ft_grasp_ref_speed

        door_hinge_lock = {hinge_joint.short_name: 0}
        jpl_hinge_lock = JointPositionList(goal_state=door_hinge_lock,
                                           name='Lock Hinge while grasp')
        jpl_hinge_lock.start_condition = self.start_condition
        self.add_task(jpl_hinge_lock)

        door_joint = PrefixName(name='iai_kitchen:arena:door_hinge_joint', prefix='iai_kitchen')
        init_vector = cas.Vector3(reference_frame=camera_link)

        if camera_link is not None and tip_push is not None:
            handle_correction = HandleOffsetCorrection(root_link=root_link,
                                                       tip_link=camera_link,
                                                       goal_vector=init_vector,
                                                       door_move_joint=door_joint,
                                                       threshold=15,
                                                       error_adjustment=500)
            handle_correction.start_condition = pre_grasp_name
            handle_correction.end_condition = handle_correction
            self.add_monitor(handle_correction)

            end_condition_pre_grasp = handle_correction

            if handle_correction_offset is not None:
                midpoint_offset = CartesianPosition(root_link=root_link,
                                                    tip_link=tip_link,
                                                    goal_point=handle_correction_offset,
                                                    name='offset handle midpoint',
                                                    threshold=0.001)
                midpoint_offset.start_condition = handle_correction
                midpoint_offset.end_condition = midpoint_offset
                self.add_task(midpoint_offset)

                next_condition = midpoint_offset
            else:
                next_condition = handle_correction
        else:
            next_condition = pre_grasp_name
            end_condition_pre_grasp = pre_grasp_name

        pre_grasp = GraspBarOffset(name=pre_grasp_name,
                                   root_link=root_link,
                                   tip_link=tip_link,
                                   tip_grasp_axis=tip_grasp_axis,
                                   bar_center=bar_center,
                                   bar_axis=bar_axis,
                                   bar_length=bar_length,
                                   grasp_axis_offset=pre_grasp_axis_offset,
                                   handle_link=handle_name)
        pre_grasp.start_condition = self.start_condition
        pre_grasp.end_condition = end_condition_pre_grasp
        self.add_task(pre_grasp)

        ap_pre_grasp = AlignPlanes(name='grasp align',
                                   root_link=root_link,
                                   tip_link=tip_link,
                                   tip_normal=tip_align_axis,
                                   goal_normal=handle_align_axis)
        ap_pre_grasp.start_condition = self.start_condition
        self.add_task(ap_pre_grasp)

        ap_tip_grasp = AlignPlanes(name='tip grasp align',
                                   root_link=root_link,
                                   tip_link=tip_link,
                                   tip_normal=tip_grasp_axis,
                                   goal_normal=bar_axis)
        ap_tip_grasp.start_condition = self.start_condition
        self.add_task(ap_tip_grasp)

        sleep_cancel = Sleep(seconds=timeout, name='ft sleep cancel')
        sleep_cancel.start_condition = next_condition
        self.add_monitor(sleep_cancel)

        ft_monitor = PayloadForceTorque(threshold_enum=ForceTorqueThresholds.DOOR.value,
                                        object_type='',
                                        name='grasp ft monitor',
                                        topic=ft_topic)
        ft_monitor.start_condition = next_condition
        self.add_monitor(ft_monitor)

        if camera_link is not None and tip_push is not None:
            ft_grasp = CartesianPosition(root_link=root_link,
                                         tip_link=tip_link,
                                         goal_point=tip_push,
                                         name='ft grasp',
                                         reference_velocity=self.reference_linear_velocity,
                                         threshold=0.001)
            ft_grasp.start_condition = next_condition
            ft_grasp.end_condition = ft_monitor
            self.add_task(ft_grasp)
        else:
            ft_grasp = GraspBarOffset(name='ft grasp',
                                      root_link=root_link,
                                      tip_link=tip_link,
                                      tip_grasp_axis=tip_grasp_axis,
                                      bar_center=bar_center,
                                      bar_axis=bar_axis,
                                      bar_length=bar_length,
                                      grasp_axis_offset=grasp_axis_offset,
                                      reference_linear_velocity=self.reference_linear_velocity,
                                      reference_angular_velocity=self.reference_angular_velocity,
                                      handle_link=handle_name)
            ft_grasp.start_condition = next_condition
            ft_grasp.end_condition = ft_monitor
            self.add_task(ft_grasp)

        retract = CartesianPosition(root_link=root_link,
                                    tip_link=tip_link,
                                    goal_point=tip_retract,
                                    name='retract after ft',
                                    reference_velocity=self.reference_linear_velocity,
                                    threshold=0.001)
        retract.start_condition = ft_monitor
        retract.end_condition = retract
        self.add_task(retract)

        ft_cancel = CancelMotion(exception=ObjectForceTorqueThresholdException('Door not touched!'),
                                 name='FT CancelMotion')
        ft_cancel.start_condition = f'not {ft_monitor} and {sleep_cancel}'
        self.add_monitor(ft_cancel)

        self.observation_expression = retract.observation_expression
