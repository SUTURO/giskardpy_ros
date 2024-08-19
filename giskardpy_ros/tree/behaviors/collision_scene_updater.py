from typing import Optional

from py_trees.common import Status

from giskardpy.god_map import god_map
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard
from line_profiler import profile


class CollisionSceneUpdater(GiskardBehavior):
    def __init__(self):
        super().__init__('update collision scene')

    @catch_and_raise_to_blackboard
    @record_time
    @profile
    def update(self):
        god_map.collision_scene.sync()
        return Status.SUCCESS
