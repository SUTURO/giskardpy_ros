from py_trees.common import Status

from giskardpy.god_map import god_map
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard
from giskardpy.utils.decorators import record_time
from line_profiler import profile


class NotifyStateChange(GiskardBehavior):

    @record_time
    @profile
    def update(self):
        god_map.world.notify_state_change()
        return Status.SUCCESS


class NotifyModelChange(GiskardBehavior):

    @catch_and_raise_to_blackboard
    @record_time
    @profile
    def update(self):
        god_map.world._notify_model_change()
        return Status.SUCCESS
