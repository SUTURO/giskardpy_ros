from py_trees.common import Status

from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from line_profiler import profile


class ClearBlackboardException(GiskardBehavior):
    @record_time
    @profile
    def update(self):
        if self.get_blackboard_exception() is not None:
            self.clear_blackboard_exception()
        return Status.SUCCESS
