#!/usr/bin/env python
import rospy
from tmc_control_msgs.msg import GripperApplyEffortActionGoal, GripperApplyEffortActionResult

rospy.init_node('gripper_sim')

echo = rospy.Publisher('/hsrb/gripper_controller/grasp/result', GripperApplyEffortActionResult,
                       queue_size=1)

rospy.loginfo('gripper simulator started')

while True:
    msg: GripperApplyEffortActionGoal = rospy.wait_for_message('/hsrb/gripper_controller/grasp/goal',
                                                               GripperApplyEffortActionGoal)
    result = GripperApplyEffortActionResult()
    result.status.goal_id = msg.goal_id

    echo.publish(result)
