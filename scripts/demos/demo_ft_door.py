#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped, PointStamped, Vector3, PoseStamped, PoseWithCovarianceStamped, \
    QuaternionStamped
from rospy import Publisher

from giskardpy.data_types.suturo_types import MoveAroundHingeAlign
from giskardpy.motion_statechart.tasks.task import WEIGHT_ABOVE_CA
from giskardpy_ros.python_interface.python_interface import GiskardWrapper


def reset():
    handle_joint1 = "iai_kitchen/iai_kitchen:arena:door_handle_joint"
    hinge_joint1 = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"
    handle_joint2 = "iai_kitchen/living_room:arena:door_handle_joint"
    hinge_joint2 = "iai_kitchen/living_room:arena:door_origin_revolute_joint"

    hinge_link1 = 'iai_kitchen/iai_kitchen:arena:door_hinge_joint'
    child_frame1 = 'iai_kitchen/iai_kitchen:arena:door_hinge'
    hinge_link2 = 'iai_kitchen/living_room:arena:door_hinge_joint'
    child_frame2 = 'iai_kitchen/living_room:arena:door_hinge'
    parent_T_hinge1 = PoseStamped()
    parent_T_hinge1.header.frame_id = 'iai_kitchen/urdf_offset'
    parent_T_hinge1.pose.position.x = 0.84
    parent_T_hinge1.pose.position.y = -1.41
    parent_T_hinge1.pose.position.z = 1.04
    parent_T_hinge1.pose.orientation.w = 1
    parent_T_hinge2 = PoseStamped()
    parent_T_hinge2.header.frame_id = 'iai_kitchen/urdf_offset'
    parent_T_hinge2.pose.position.x = 0.44
    parent_T_hinge2.pose.position.y = 2.37
    parent_T_hinge2.pose.position.z = 1.04
    parent_T_hinge2.pose.orientation.w = 1

    joint_reset = gis.monitors.add_joint_position(goal_state={handle_joint1: 0,
                                                              hinge_joint1: 0,
                                                              handle_joint2: 0,
                                                              hinge_joint2: 0})
    gis.motion_goals.add_joint_position(goal_state={handle_joint1: 0,
                                                    hinge_joint1: 0,
                                                    handle_joint2: 0,
                                                    hinge_joint2: 0})

    gis.monitors.add_handle_offset_reset(reset_joint=hinge_link1, parent_T_child=parent_T_hinge1,
                                         child_frame=child_frame1, name='hinge1')
    gis.monitors.add_handle_offset_reset(reset_joint=hinge_link2, parent_T_child=parent_T_hinge2,
                                         child_frame=child_frame2, name='hinge2')

    gis.monitors.add_end_motion(start_condition=f'hinge1 and hinge2 and {joint_reset}')
    gis.motion_goals.allow_all_collisions()
    gis.execute()


def setup_door1(init_pose_pub: Publisher):
    base_pose = PoseStamped()
    base_pose.header.frame_id = 'map'
    base_pose.pose.position.x = 1.8
    base_pose.pose.position.y = -0.8
    base_pose.pose.orientation.z = -1

    odom = gis.monitors.add_local_minimum_reached()
    gis.motion_goals.add_cartesian_pose(root_link='map', tip_link='base_footprint', goal_pose=base_pose)

    gis.monitors.add_end_motion(start_condition=f'{odom}')
    gis.motion_goals.allow_all_collisions()
    gis.execute()

    gis.motion_goals.add_take_pose(pose_keyword='park')
    joints = gis.monitors.add_joint_position(goal_state={'head_pan_joint': 0.0,
                                                         'head_tilt_joint': 0.0,
                                                         'arm_lift_joint': 0.0,
                                                         'arm_flex_joint': 0.0,
                                                         'arm_roll_joint': -1.5,
                                                         'wrist_flex_joint': -1.5,
                                                         'wrist_roll_joint': 0.0},
                                             threshold=0.05)
    gis.monitors.add_end_motion(start_condition=joints)
    gis.motion_goals.allow_all_collisions()
    gis.execute()

    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.pose.pose = base_pose.pose
    init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    init_pose_pub.publish(init_pose)

    rot_left = QuaternionStamped()
    rot_left.header.frame_id = 'base_footprint'
    rot_left.quaternion.z = 0.643
    rot_left.quaternion.w = 0.766

    starting_rot = QuaternionStamped()
    starting_rot.header.frame_id = 'base_footprint'
    starting_rot.quaternion.z = -0.643
    starting_rot.quaternion.w = 0.766

    rot_left_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=rot_left,
                                                              root_link='map',
                                                              tip_link='base_footprint',
                                                              name='rotation left monitor')
    rot_start_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=starting_rot,
                                                               root_link='map',
                                                               tip_link='base_footprint',
                                                               start_condition=rot_left_monitor,
                                                               threshold=0.03,
                                                               name='rotation start monitor')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=rot_left,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               end_condition=rot_left_monitor,
                                               name='rotation left goal')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=starting_rot,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               start_condition=rot_left_monitor,
                                               end_condition=rot_start_monitor,
                                               name='rotation start goal')

    gis.monitors.add_end_motion(start_condition=rot_start_monitor)
    gis.execute()


def setup_door2(init_pose_pub: Publisher):
    base_pose = PoseStamped()
    base_pose.header.frame_id = 'map'
    base_pose.pose.position.x = 1.8
    base_pose.pose.position.y = 3.0
    base_pose.pose.orientation.z = -1

    odom = gis.monitors.add_local_minimum_reached()
    gis.motion_goals.add_cartesian_pose(root_link='map', tip_link='base_footprint', goal_pose=base_pose)

    gis.monitors.add_end_motion(start_condition=f'{odom}')
    gis.motion_goals.allow_all_collisions()
    gis.execute()

    gis.motion_goals.add_take_pose(pose_keyword='park')
    joints = gis.monitors.add_joint_position(goal_state={'head_pan_joint': 0.0,
                                                         'head_tilt_joint': 0.0,
                                                         'arm_lift_joint': 0.0,
                                                         'arm_flex_joint': 0.0,
                                                         'arm_roll_joint': -1.5,
                                                         'wrist_flex_joint': -1.5,
                                                         'wrist_roll_joint': 0.0},
                                             threshold=0.05)
    gis.monitors.add_end_motion(start_condition=joints)
    gis.motion_goals.allow_all_collisions()
    gis.execute()

    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.pose.pose = base_pose.pose
    init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    init_pose_pub.publish(init_pose)

    rot_left = QuaternionStamped()
    rot_left.header.frame_id = 'base_footprint'
    rot_left.quaternion.z = 0.643
    rot_left.quaternion.w = 0.766

    starting_rot = QuaternionStamped()
    starting_rot.header.frame_id = 'base_footprint'
    starting_rot.quaternion.z = -0.643
    starting_rot.quaternion.w = 0.766

    rot_left_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=rot_left,
                                                              root_link='map',
                                                              tip_link='base_footprint',
                                                              name='rotation left monitor')
    rot_start_monitor = gis.monitors.add_cartesian_orientation(goal_orientation=starting_rot,
                                                               root_link='map',
                                                               tip_link='base_footprint',
                                                               start_condition=rot_left_monitor,
                                                               threshold=0.03,
                                                               name='rotation start monitor')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=rot_left,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               end_condition=rot_left_monitor,
                                               name='rotation left goal')
    gis.motion_goals.add_cartesian_orientation(goal_orientation=starting_rot,
                                               root_link='map',
                                               tip_link='base_footprint',
                                               start_condition=rot_left_monitor,
                                               end_condition=rot_start_monitor,
                                               name='rotation start goal')

    gis.monitors.add_end_motion(start_condition=rot_start_monitor)
    gis.execute()


def grasping(with_camera: bool = False):
    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    hinge_joint = "iai_kitchen/iai_kitchen:arena:door_origin_revolute_joint"
    # handle_name = "iai_kitchen/living_room:arena:door_handle_inside"
    # hinge_joint = "iai_kitchen/living_room:arena:door_origin_revolute_joint"
    tip = 'hand_gripper_tool_frame'
    if with_camera:
        camera_link = 'hand_camera_frame'
    else:
        camera_link = None
    handle_length = 0.01
    ref_speed = 0.5
    handle_retract_distance = -0.058
    # handle_retract_distance = -0.065
    bar_center_offset = 0.01
    pre_grasp_distance = -0.15
    grasp_into_distance = 0.2
    ft_timeout = 10

    bar_axis = Vector3Stamped()
    bar_axis.header.frame_id = handle_name
    bar_axis.vector = Vector3(0, 1, 0)

    tip_grasp_axis = Vector3Stamped()
    tip_grasp_axis.header.frame_id = tip
    tip_grasp_axis.vector = Vector3(1, 0, 0)

    bar_center = PointStamped()
    bar_center.header.frame_id = handle_name

    x_gripper = Vector3Stamped()
    x_gripper.header.frame_id = tip
    x_gripper.vector.z = 1

    x_goal = Vector3Stamped()
    x_goal.header.frame_id = handle_name
    x_goal.vector.z = -1

    grasp_axis_offset = Vector3Stamped()
    grasp_axis_offset.header.frame_id = handle_name
    grasp_axis_offset.vector.z = grasp_into_distance

    pre_grasp_axis_offset = Vector3Stamped()
    pre_grasp_axis_offset.header.frame_id = handle_name
    pre_grasp_axis_offset.vector.z = pre_grasp_distance

    handle_retract = PointStamped()
    handle_retract.header.frame_id = tip
    handle_retract.point.z = handle_retract_distance

    grasp_push = PointStamped()
    grasp_push.header.frame_id = tip
    grasp_push.point.z = grasp_into_distance - pre_grasp_distance

    js = {
        'head_pan_joint': 0,
        'head_tilt_joint': 0,
        'arm_flex_joint': 0,
        'arm_roll_joint': 0,
        'wrist_flex_joint': -np.pi / 2,
        'wrist_roll_joint': -np.pi / 2
    }
    jps = gis.motion_goals.add_joint_position(goal_state=js,
                                              name='hold fixed grasping position',
                                              threshold=0.05)

    open_gripper = gis.monitors.add_open_hsr_gripper(start_condition=jps)

    handle_correction_offset = PointStamped()
    handle_correction_offset.header.frame_id = tip
    handle_correction_offset.point.x = 0.03

    grasp = gis.motion_goals.add_grasp_with_ft_sensor(root_link='map',
                                                      tip_link=tip,
                                                      handle_name=handle_name,
                                                      tip_grasp_axis=tip_grasp_axis,
                                                      bar_axis=bar_axis,
                                                      tip_retract=handle_retract,
                                                      handle_align_axis=x_goal,
                                                      tip_align_axis=x_gripper,
                                                      grasp_axis_offset=grasp_axis_offset,
                                                      pre_grasp_axis_offset=pre_grasp_axis_offset,
                                                      hinge_joint=hinge_joint,
                                                      timeout=ft_timeout,
                                                      ft_grasp_ref_speed=ref_speed,
                                                      camera_link=camera_link,
                                                      tip_push=grasp_push,
                                                      handle_correction_offset=handle_correction_offset,
                                                      start_condition=open_gripper)
    gis.update_end_condition(node_name=grasp, condition=grasp)

    close_gripper = gis.monitors.add_close_hsr_gripper(start_condition=grasp)

    gis.monitors.add_end_motion(start_condition=close_gripper)

    gis.motion_goals.allow_all_collisions()
    gis.execute()


def full_opening():
    grasping()

    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    door_handle_for_hinge = "iai_kitchen/iai_kitchen:arena:door_handle_link"
    door_center = "iai_kitchen/iai_kitchen:arena:door_center"
    # handle_name = "iai_kitchen/living_room:arena:door_handle_inside"
    # door_handle_for_hinge = "iai_kitchen/living_room:arena:door_handle_link"
    # door_center = "iai_kitchen/living_room:arena:door_center"
    handle_turn_limit = 0.4
    pre_push_hinge_turn_limit = -0.5
    full_hinge_turn_limit = -1.4
    open_door_name = 'OpenDoorGoal'
    tip = 'hand_gripper_tool_frame'
    root = 'map'
    tip_grasp_axis = Vector3Stamped()
    tip_grasp_axis.header.frame_id = tip
    tip_grasp_axis.vector = Vector3(1, 0, 0)
    move_around_name = 'MovingAroundAtTheSpeedOfSound'
    multipliers = [(7 / 5, -0.3, 'down_long'),
                   (7 / 5, 0.4, 'up_long'),
                   (0.8, 0.7, 'up_short')]

    offset = Vector3Stamped()
    offset.header.frame_id = root
    offset.vector = Vector3(0, 0, -0.15)

    retract_name = 'retract_after_ft'
    handle_retract_distance = -0.15
    handle_retract = PointStamped()
    handle_retract.header.frame_id = tip
    handle_retract.point.z = handle_retract_distance
    goal_orientation = QuaternionStamped()
    goal_orientation.header.frame_id = tip
    goal_orientation.quaternion.w = 1

    open_goal = gis.motion_goals.hsrb_open_door_goal(handle_name=handle_name,
                                                     handle_limit=handle_turn_limit,
                                                     hinge_limit=pre_push_hinge_turn_limit,
                                                     name=open_door_name,
                                                     end_condition=open_door_name)

    slep = gis.monitors.add_sleep(seconds=0.5, start_condition=open_goal)

    open_gripper = gis.monitors.add_open_hsr_gripper(start_condition=slep)

    gis.motion_goals.add_cartesian_orientation(goal_orientation=goal_orientation,
                                               root_link=root,
                                               tip_link=tip,
                                               name='fix_rotation_on_retract',
                                               start_condition=slep,
                                               end_condition=retract_name)

    open_goal_retract = gis.motion_goals.add_cartesian_position(root_link=root,
                                                                tip_link=tip,
                                                                goal_point=handle_retract,
                                                                name=retract_name,
                                                                threshold=0.001,
                                                                start_condition=open_gripper,
                                                                end_condition=retract_name)

    open_goal_moving_around = gis.motion_goals.add_move_around_hinge(handle_name=door_handle_for_hinge,
                                                                     tip_gripper_axis=tip_grasp_axis,
                                                                     root_link=root,
                                                                     tip_link=tip,
                                                                     goal_angle=pre_push_hinge_turn_limit,
                                                                     multipliers=multipliers,
                                                                     offset=offset,
                                                                     align_gripper=MoveAroundHingeAlign.ALL,
                                                                     name=move_around_name,
                                                                     start_condition=open_goal_retract,
                                                                     end_condition=move_around_name)

    close_gripper = gis.monitors.add_close_hsr_gripper(start_condition=open_goal_moving_around)

    pre_push = gis.motion_goals.add_pre_push_door(root_link=root,
                                                  tip_link=tip,
                                                  door_handle=door_handle_for_hinge,
                                                  weight=WEIGHT_ABOVE_CA,
                                                  door_object=door_center,
                                                  start_condition=close_gripper)
    gis.update_end_condition(pre_push, pre_push)

    open_full = gis.motion_goals.add_open_container(tip_link=tip,
                                                    environment_link=door_center,
                                                    name='Push open door',
                                                    goal_joint_state=full_hinge_turn_limit,
                                                    start_condition=pre_push)
    gis.update_end_condition(open_full, open_full)

    gis.motion_goals.allow_collision(group1='arm',
                                     group2='iai_kitchen')
    gis.motion_goals.avoid_all_collisions(start_condition=open_goal_retract)
    gis.motion_goals.allow_collision(group1='arm',
                                     group2='iai_kitchen',
                                     start_condition=open_goal_moving_around)
    # gis.motion_goals.allow_all_collisions()
    gis.monitors.add_end_motion(start_condition=open_full)
    gis.execute()


def full_opening_in_parts():
    grasping(True)

    handle_name = "iai_kitchen/iai_kitchen:arena:door_handle_inside"
    door_handle_for_hinge = "iai_kitchen/iai_kitchen:arena:door_handle_link"
    door_center = "iai_kitchen/iai_kitchen:arena:door_center"
    # handle_name = "iai_kitchen/living_room:arena:door_handle_inside"
    # door_handle_for_hinge = "iai_kitchen/living_room:arena:door_handle_link"
    # door_center = "iai_kitchen/living_room:arena:door_center"
    handle_turn_limit = 0.4
    # handle_turn_limit = 0.55
    pre_push_hinge_turn_limit = -0.5
    full_hinge_turn_limit = -1.4
    # full_hinge_turn_limit = -1.0
    open_door_name = 'OpenDoorGoal'
    tip = 'hand_gripper_tool_frame'
    root = 'map'
    tip_grasp_axis = Vector3Stamped()
    tip_grasp_axis.header.frame_id = tip
    tip_grasp_axis.vector = Vector3(1, 0, 0)
    move_around_name = 'MovingAroundAtTheSpeedOfSound'
    multipliers = [(7 / 5, -0.3, 'down_long'),
                   (7 / 5, 0.4, 'up_long'),
                   (0.8, 0.7, 'up_short')]

    offset = Vector3Stamped()
    offset.header.frame_id = root
    offset.vector = Vector3(0, 0, -0.15)

    retract_name = 'retract_after_ft'
    handle_retract_distance = -0.15
    handle_retract = PointStamped()
    handle_retract.header.frame_id = tip
    handle_retract.point.z = handle_retract_distance
    goal_orientation = QuaternionStamped()
    goal_orientation.header.frame_id = tip
    goal_orientation.quaternion.w = 1

    open_goal = gis.motion_goals.hsrb_open_door_goal(handle_name=handle_name,
                                                     handle_limit=handle_turn_limit,
                                                     hinge_limit=pre_push_hinge_turn_limit,
                                                     name=open_door_name,
                                                     end_condition=open_door_name)

    slep = gis.monitors.add_sleep(seconds=0.5, start_condition=open_goal)

    open_gripper = gis.monitors.add_open_hsr_gripper(start_condition=slep)

    gis.motion_goals.add_cartesian_orientation(goal_orientation=goal_orientation,
                                               root_link=root,
                                               tip_link=tip,
                                               name='fix_rotation_on_retract',
                                               start_condition=slep,
                                               end_condition=retract_name)

    open_goal_retract = gis.motion_goals.add_cartesian_position(root_link=root,
                                                                tip_link=tip,
                                                                goal_point=handle_retract,
                                                                name=retract_name,
                                                                threshold=0.001,
                                                                start_condition=open_gripper,
                                                                end_condition=retract_name)

    gis.motion_goals.allow_collision(group1='arm',
                                     group2='iai_kitchen')
    gis.monitors.add_end_motion(open_goal_retract)
    gis.execute()

    open_goal_moving_around = gis.motion_goals.add_move_around_hinge(handle_name=door_handle_for_hinge,
                                                                     tip_gripper_axis=tip_grasp_axis,
                                                                     root_link=root,
                                                                     tip_link=tip,
                                                                     goal_angle=pre_push_hinge_turn_limit,
                                                                     multipliers=multipliers,
                                                                     offset=offset,
                                                                     align_gripper=MoveAroundHingeAlign.ALL,
                                                                     name=move_around_name,
                                                                     start_condition='',
                                                                     end_condition=move_around_name)

    gis.motion_goals.avoid_all_collisions()
    gis.monitors.add_end_motion(move_around_name)
    gis.execute()

    close_gripper = gis.monitors.add_close_hsr_gripper()

    pre_push = gis.motion_goals.add_pre_push_door(root_link=root,
                                                  tip_link=tip,
                                                  door_handle=door_handle_for_hinge,
                                                  weight=WEIGHT_ABOVE_CA,
                                                  door_object=door_center,
                                                  start_condition=close_gripper)
    gis.update_end_condition(pre_push, pre_push)

    open_full = gis.motion_goals.add_open_container(tip_link=tip,
                                                    environment_link=door_center,
                                                    name='Push open door',
                                                    goal_joint_state=full_hinge_turn_limit,
                                                    start_condition=pre_push)
    gis.update_end_condition(open_full, open_full)

    gis.motion_goals.allow_collision(group1='arm',
                                     group2='iai_kitchen')
    # gis.motion_goals.allow_all_collisions()
    gis.monitors.add_end_motion(start_condition=open_full)
    gis.execute()


rospy.init_node('giskard_demo')

init_pub = rospy.Publisher('/initialpose', data_class=PoseWithCovarianceStamped, queue_size=10)

gis = GiskardWrapper()
test = 3

reset()

setup_door1(init_pose_pub=init_pub)

# input("Setup finished?")

if test == 1:
    full_opening()
elif test == 2:
    grasping(True)
elif test == 3:
    full_opening_in_parts()
else:
    gis.hsr_door_opening(ft_timeout=1000)
