#!/usr/bin/env python
import rospy
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy.utils.utils import get_middleware

if __name__ == '__main__':
    rospy.init_node('clear_world')
    giskard = GiskardWrapper()
    result = giskard.world.clear()
    if result.error_codes == result.SUCCESS:
        get_middleware().loginfo('World cleared.')
    else:
        get_middleware().logwarn(f'Failed to clear world {result}.')
