import os

if 'GITHUB_WORKFLOW' not in os.environ:
    from typing import Optional

    import actionlib

    from giskardpy import casadi_wrapper as cas
    from giskardpy.monitors.monitors import PayloadMonitor

    from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal


    class MoveHSRGripper(PayloadMonitor):
        action_server: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/hsrb/gripper_controller/grasp',
                                                                                   GripperApplyEffortAction)

        def __init__(self, *,
                     name: Optional[str] = None,
                     force: float,
                     start_condition: cas.Expression = cas.TrueSymbol):
            super().__init__(run_call_in_thread=True,
                             name=name,
                             stay_true=True,
                             start_condition=start_condition)
            self.force = force

        def __call__(self):
            goal = GripperApplyEffortGoal()
            goal.effort = self.force
            self.action_server.send_goal_and_wait(goal)
            self.state = True


    class OpenHsrGripper(MoveHSRGripper):
        def __init__(self, *,
                     name: Optional[str] = None,
                     start_condition: cas.Expression = cas.TrueSymbol):
            super().__init__(name=name, force=0.8, start_condition=start_condition)


    class CloseHsrGripper(MoveHSRGripper):
        def __init__(self, *,
                     name: Optional[str] = None,
                     start_condition: cas.Expression = cas.TrueSymbol):
            super().__init__(name=name, force=-0.8, start_condition=start_condition)
