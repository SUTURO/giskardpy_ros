import pytest
import rclpy
from geometry_msgs.msg import PoseStamped

import giskardpy_ros.ros2.tfwrapper as tf
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.model.joints import OneDofJoint
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard

from giskardpy_ros.utils.utils_for_tests import GiskardTester


@pytest.fixture(scope='module')
def ros(request):
    rospy.init_node('giskard')
    get_middleware().loginfo('init ros')
    tf.init()
    get_middleware().loginfo('done tf init')

    def kill_ros():
        try:
            GiskardBlackboard().tree.render()
        except KeyError as e:
            get_middleware().logerr(f'Failed to render behavior tree.')
        get_middleware().loginfo('shutdown ros')
        rclpy.shutdown()

    # try:
    #     rospy.get_param('kitchen_description')
    # except:
    #     try:
    #         launch_launchfile('package://iai_kitchen/launch/upload_kitchen_obj.launch')
    #     except:
    #         middleware.logwarn('iai_apartment not found')
    # try:
    #     rospy.get_param('apartment_description')
    # except:
    #     try:
    #         launch_launchfile('package://iai_apartment/launch/upload_apartment.launch')
    #     except:
    #         middleware.logwarn('iai_kitchen not found')
    request.addfinalizer(kill_ros)


@pytest.fixture()
def resetted_giskard(giskard: GiskardTester) -> GiskardTester:
    get_middleware().loginfo('resetting giskard')
    giskard.api.clear_motion_goals_and_monitors()
    if GiskardBlackboard().tree.is_standalone() and giskard.has_odometry_joint():
        zero = PoseStamped()
        zero.header.frame_id = 'map'
        zero.pose.orientation.w = 1
        done = giskard.api.monitors.add_set_seed_odometry(zero)
        giskard.api.motion_goals.allow_all_collisions()
        giskard.api.monitors.add_end_motion(start_condition=done)
        giskard.execute(add_local_minimum_reached=False)
    giskard.api.world.clear()
    giskard.reset()
    return giskard


@pytest.fixture()
def zero_pose(resetted_giskard: GiskardTester) -> GiskardTester:
    if GiskardBlackboard().tree.is_standalone():
        done = resetted_giskard.api.monitors.add_set_seed_configuration(resetted_giskard.default_pose)
        resetted_giskard.api.motion_goals.allow_all_collisions()
        resetted_giskard.api.monitors.add_end_motion(start_condition=done)
        resetted_giskard.execute(add_local_minimum_reached=False)
    else:
        resetted_giskard.api.motion_goals.allow_all_collisions()
        resetted_giskard.api.motion_goals.add_joint_position(resetted_giskard.default_pose)
        resetted_giskard.api.add_default_end_motion_conditions()
        resetted_giskard.execute()
    return resetted_giskard


@pytest.fixture()
def better_pose(resetted_giskard: GiskardTester) -> GiskardTester:
    if GiskardBlackboard().tree.is_standalone():
        done = resetted_giskard.api.monitors.add_set_seed_configuration(resetted_giskard.better_pose)
        resetted_giskard.api.motion_goals.allow_all_collisions()
        resetted_giskard.api.monitors.add_end_motion(start_condition=done)
        resetted_giskard.execute(add_local_minimum_reached=False)
    else:
        resetted_giskard.api.motion_goals.allow_all_collisions()
        resetted_giskard.api.motion_goals.add_joint_position(resetted_giskard.better_pose)
        resetted_giskard.api.add_default_end_motion_conditions()
        resetted_giskard.execute()
    return resetted_giskard


@pytest.fixture()
def kitchen_setup(better_pose: GiskardTester) -> GiskardTester:
    better_pose.default_env_name = 'iai_kitchen'
    if GiskardBlackboard().tree.is_standalone():
        kitchen_pose = PoseStamped()
        kitchen_pose.header.frame_id = str(better_pose.default_root)
        kitchen_pose.pose.orientation.w = 1
        better_pose.add_urdf_to_world(name=better_pose.default_env_name,
                                      urdf=rospy.get_param('kitchen_description'),
                                      pose=kitchen_pose)
    else:
        kitchen_pose = tf.lookup_pose('map', 'iai_kitchen/world')
        better_pose.add_urdf_to_world(name=better_pose.default_env_name,
                                      urdf=rospy.get_param('kitchen_description'),
                                      pose=kitchen_pose,
                                      js_topic='/kitchen/joint_states',
                                      set_js_topic='/kitchen/cram_joint_states')
    js = {}
    for joint_name in god_map.world.groups[better_pose.default_env_name].movable_joint_names:
        joint = god_map.world.joints[joint_name]
        if isinstance(joint, OneDofJoint):
            if GiskardBlackboard().tree.is_standalone():
                js[str(joint.free_variable.name)] = 0.0
            else:
                js[str(joint.free_variable.name.short_name)] = 0.0
    better_pose.set_env_state(js)
    return better_pose


@pytest.fixture()
def apartment_setup(better_pose: GiskardTester) -> GiskardTester:
    better_pose.default_env_name = 'iai_apartment'
    if GiskardBlackboard().tree.is_standalone():
        kitchen_pose = PoseStamped()
        kitchen_pose.header.frame_id = str(better_pose.default_root)
        kitchen_pose.pose.orientation.w = 1
        better_pose.add_urdf_to_world(name=better_pose.default_env_name,
                                      urdf=rospy.get_param('apartment_description'),
                                      pose=kitchen_pose)
    else:
        better_pose.add_urdf_to_world(name=better_pose.default_env_name,
                                      urdf=rospy.get_param('apartment_description'),
                                      pose=tf.lookup_pose('map', 'iai_apartment/apartment_root'),
                                      js_topic='/apartment_joint_states',
                                      set_js_topic='/iai_kitchen/cram_joint_states')
    js = {}
    for joint_name in god_map.world.groups[better_pose.default_env_name].movable_joint_names:
        joint = god_map.world.joints[joint_name]
        if isinstance(joint, OneDofJoint):
            js[str(joint.free_variable.name)] = 0.0
    better_pose.set_env_state(js)
    base_pose = PoseStamped()
    base_pose.header.frame_id = 'iai_apartment/side_B'
    base_pose.pose.position.x = 1.5
    base_pose.pose.position.y = 2.4
    base_pose.pose.orientation.w = 1
    base_pose = better_pose.transform_msg(god_map.world.root_link_name, base_pose)
    better_pose.teleport_base(base_pose)
    return better_pose
