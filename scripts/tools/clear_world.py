#!/usr/bin/env python
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy.utils import logging

if __name__ == '__main__':
    rospy.init_node('clear_world')
    giskard = GiskardWrapper()
    result = giskard.clear_world()
    if result.error_codes == result.SUCCESS:
        logging.loginfo('World cleared.')
    else:
        logging.logwarn(f'Failed to clear world {result}.')
