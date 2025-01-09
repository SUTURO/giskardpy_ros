import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerResponse
from line_profiler import profile
from py_trees import Status
from visualization_msgs.msg import MarkerArray, Marker

from giskardpy.god_map import god_map
from giskardpy.model.collision_world_syncer import Collisions
from giskardpy.utils.decorators import record_time
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard, GiskardBlackboard


class CleanUp(GiskardBehavior):
    @profile
    def __init__(self, name, clear_markers=False):
        super().__init__(name)
        self.clear_markers_ = clear_markers
        self.marker_pub = rospy.Publisher('~visualization_marker_array', MarkerArray, queue_size=10)

    def clear_markers(self):
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        self.marker_pub.publish(msg)

    @record_time
    @profile
    def initialise(self):
        if self.clear_markers_:
            self.clear_markers()
        if GiskardBlackboard().tree.control_loop_branch.publish_state.debug_marker_publisher is not None:
            self.clear_markers()
            GiskardBlackboard().ros_visualizer.publish_markers(force=True)
        GiskardBlackboard().giskard.set_defaults()
        god_map.world.compiled_all_fks = None
        god_map.collision_scene.reset_cache()
        god_map.collision_scene.clear_collision_matrix()
        god_map.closest_point = Collisions(1)
        god_map.time = 0
        god_map.control_cycle_counter = 1
        god_map.monitor_manager.reset()
        god_map.motion_goal_manager.reset()
        god_map.debug_expression_manager.reset()

        if hasattr(self.get_blackboard(), 'runtime'):
            del self.get_blackboard().runtime

    def update(self):
        return Status.SUCCESS


class CleanUpPlanning(CleanUp):
    def initialise(self):
        super().initialise()
        GiskardBlackboard().fill_trajectory_velocity_values = None
        god_map.free_variables = []

    @catch_and_raise_to_blackboard
    def update(self):
        return super().update()


class ActivateHSRControllers(GiskardBehavior):
    def initialise(self):
        super().initialise()
        # Setup Services
        rospy.wait_for_service('/hsrb/controller_manager/switch_controller')
        GiskardBlackboard().controller_manager = rospy.ServiceProxy(name='/hsrb/controller_manager/switch_controller',
                                                                    service_class=SwitchController)
        # rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        # GiskardBlackboard().list_controller = rospy.ServiceProxy(name='/hsrb/controller_manager/list_controllers',
        #                                                          service_class=ListControllers)

    @catch_and_raise_to_blackboard
    def update(self):
        # if GiskardBlackboard().controller_manager is not None and GiskardBlackboard().list_controller is not None:
        if GiskardBlackboard().controller_manager is not None:
            resp: SwitchControllerResponse = GiskardBlackboard().controller_manager(['realtime_body_controller_real'],
                                                                                    ['arm_trajectory_controller',
                                                                                     'head_trajectory_controller'],
                                                                                    2, False, 0.0)
        return Status.SUCCESS


class DeactivateHSRControllers(GiskardBehavior):
    def initialise(self):
        super().initialise()

    @catch_and_raise_to_blackboard
    def update(self):
        # if GiskardBlackboard().controller_manager is not None and GiskardBlackboard().list_controller is not None:
        if GiskardBlackboard().controller_manager is not None:
            resp: SwitchControllerResponse = GiskardBlackboard().controller_manager(['arm_trajectory_controller',
                                                                                     'head_trajectory_controller'],
                                                                                    ['realtime_body_controller_real'],
                                                                                    2, False, 0.0)
        return Status.SUCCESS
