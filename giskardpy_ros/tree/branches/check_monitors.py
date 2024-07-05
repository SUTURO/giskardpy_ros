from py_trees.decorators import SuccessIsRunning

from giskardpy.motion_graph.monitors.monitors import EndMotion
from giskardpy.motion_graph.monitors.payload_monitors import PayloadMonitor
from giskardpy_ros.tree.branches.payload_monitor_sequence import PayloadMonitorSequence
from giskardpy_ros.tree.composites.running_selector import RunningSelector


class CheckMonitors(RunningSelector):

    def __init__(self, name: str = 'check monitors'):
        super().__init__(name, memory=False)

    def add_monitor(self, monitor: PayloadMonitor):
        if isinstance(monitor, EndMotion):
            self.add_child(PayloadMonitorSequence(monitor))
        else:
            payload_monitor_sequence = PayloadMonitorSequence(monitor)
            self.add_child(SuccessIsRunning('success is running', payload_monitor_sequence))
