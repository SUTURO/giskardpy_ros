from queue import Queue, Empty
from typing import Any, Optional

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.timer import Timer

from giskard_msgs.action import Move
from giskardpy.data_types.exceptions import GiskardException
from giskardpy.middleware import get_middleware
from giskardpy.utils.decorators import record_time
from giskardpy_ros.ros2 import rospy


class ActionServerHandler:
    """
    Interface to action server which is more useful for behaviors.
    """
    goal_id: int
    name: str
    client_alive_checker: Timer
    client_alive: bool
    cancel_requested: bool = False
    goal_handle: Optional[ServerGoalHandle]

    @record_time
    def __init__(self, action_name: str, action_type: Any):
        self.name = action_name
        self.goal_id = -1
        self.goal_msg = None
        self._result_msg = None
        self.client_alive_checker = None
        self.goal_queue = Queue(1)
        self.result_queue = Queue(1)
        self.goal_handle = None
        self._as = ActionServer(node=rospy.node,
                                action_type=action_type,
                                action_name=self.name,
                                execute_callback=self.execute_cb,
                                goal_callback=self.default_goal_callback,
                                cancel_callback=self.cancel_callback)

    def default_goal_callback(self, goal_request):
        if self.goal_handle is not None:
            get_middleware().loginfo('cancelling old goal')
            self.cancel_requested = True
        get_middleware().loginfo('new goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        get_middleware().loginfo('Cancel request received')
        return CancelResponse.ACCEPT

    def is_goal_msg_type_execute(self):
        return self.goal_msg.type in [Move.Goal.EXECUTE]

    def is_goal_msg_type_projection(self):
        return Move.Goal.PROJECTION == self.goal_msg.type

    def is_goal_msg_type_undefined(self):
        return Move.Goal.UNDEFINED == self.goal_msg.type

    async def execute_cb(self, goal: ServerGoalHandle) -> None:
        self.goal_queue.put(goal)
        result_msg = self.result_queue.get()
        get_middleware().loginfo('sending response')
        # self.client_alive_checker.shutdown()
        self.goal_msg = None
        self.goal_handle = None
        self.result_msg = None
        self.cancel_requested = False
        self.payload()
        return result_msg

    def is_client_alive(self) -> bool:
        return True

    @profile
    def ping_client(self, time):
        client_name = self._as.current_goal.goal.goal_id.id.split('-')[0]
        self.client_alive = rospy.node.rosnode_ping(client_name, max_count=1)
        if not self.client_alive:
            get_middleware().logerr(f'Lost connection to Client "{client_name}".')
            self.client_alive_checker.shutdown()

    def accept_goal(self) -> None:
        try:
            self.goal_handle = self.goal_queue.get_nowait()
            self.goal_msg = self.goal_handle.request
            # self.client_alive = True
            # self.client_alive_checker = rospy.node.create_timer(1.0, callback=self.ping_client)
            self.goal_id += 1
        except Empty:
            return None

    @property
    def result_msg(self):
        if self._result_msg is None:
            raise GiskardException('no result message set.')
        return self._result_msg

    @result_msg.setter
    def result_msg(self, value):
        self._result_msg = value

    def has_goal(self):
        return not self.goal_queue.empty()

    def send_feedback(self, message):
        self.goal_handle.publish_feedback(message)

    def set_canceled(self):
        self.payload = self.goal_handle.canceled

    def set_aborted(self):
        self.payload = self.goal_handle.abort

    def set_succeeded(self):
        self.payload = self.goal_handle.succeed

    def send_result(self):
        self.result_queue.put(self.result_msg)

    def is_cancel_requested(self) -> bool:
        return self.cancel_requested or self.goal_handle.is_cancel_requested
