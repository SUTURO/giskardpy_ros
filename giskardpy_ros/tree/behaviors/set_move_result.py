from giskard_msgs.action import Move
from py_trees.common import Status

from giskard_msgs.msg import GiskardError
from giskardpy.data_types.exceptions import *
from giskardpy.god_map import god_map
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.middleware import get_middleware
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from giskardpy.utils.decorators import record_time
import giskardpy_ros.ros2.msg_converter as msg_converter


class SetMoveResult(GiskardBehavior):

    @profile
    def __init__(self, name, context, print=True):
        self.print = print
        self.context = context
        super().__init__(name)

    @record_time
    @profile
    def update(self):
        e = self.get_blackboard_exception()
        if e is None:
            move_result = Move.Result()
            GiskardBlackboard().move_action_server.set_succeeded()
        else:
            move_result = Move.Result(error=msg_converter.exception_to_error_msg(e))
            GiskardBlackboard().move_action_server.set_aborted()

        trajectory = god_map.trajectory
        joints = [god_map.world.joints[joint_name] for joint_name in god_map.world.movable_joint_names]
        sample_period = god_map.qp_controller.sample_period
        move_result.trajectory = msg_converter.trajectory_to_ros_trajectory(trajectory,
                                                                            sample_period=sample_period,
                                                                            start_time=0,
                                                                            joints=joints)
        if isinstance(e, PreemptedException):
            get_middleware().logwarn(f'Goal preempted: \'{move_result.error.msg}\'.')
        else:
            if self.print:
                if move_result.error.type == GiskardError.SUCCESS:
                    get_middleware().loginfo(f'{self.context} succeeded.')
                else:
                    get_middleware().logwarn(f'{self.context} failed: {move_result.error.msg}.')
        GiskardBlackboard().move_action_server.result_msg = move_result
        return Status.SUCCESS
