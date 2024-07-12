from py_trees.composites import Sequence

from giskard_msgs.action import World
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.action_server import ActionServerHandler
from giskardpy_ros.tree.behaviors.collision_scene_updater import CollisionSceneUpdater
from giskardpy_ros.tree.behaviors.goal_received import GoalReceived
from giskardpy_ros.tree.behaviors.notify_state_change import NotifyStateChange, NotifyModelChange
from giskardpy_ros.tree.behaviors.send_result import SendResult
from giskardpy_ros.tree.behaviors.world_updater import ProcessWorldUpdate
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from giskardpy_ros.tree.branches.publish_state import PublishState
from giskardpy_ros.tree.branches.synchronization import Synchronization


class UpdateWorld(Sequence):
    synchronization: Synchronization
    publish_state: PublishState
    goal_received: GoalReceived
    process_goal: ProcessWorldUpdate

    def __init__(self):
        name = 'update world'
        super().__init__(name, memory=True)
        GiskardBlackboard().world_action_server = ActionServerHandler(action_name=f'{rospy.node.get_name()}/update_world',
                                                                      action_type=World)
        self.goal_received = GoalReceived(GiskardBlackboard().world_action_server)
        self.send_result = SendResult(action_server=GiskardBlackboard().world_action_server)
        self.process_goal = ProcessWorldUpdate(action_server=GiskardBlackboard().world_action_server)

        self.add_child(self.goal_received)
        self.add_child(self.process_goal)
        self.add_child(NotifyModelChange())
        self.add_child(CollisionSceneUpdater())
        self.add_child(self.send_result)
