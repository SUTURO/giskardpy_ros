import rospy
import numpy as np
from giskardpy_ros.python_interface.python_interface import GiskardWrapper

rospy.init_node('adsfasdf')

giskard = GiskardWrapper()

giskard.motion_goals.add_carry_my_luggage(name='cmb', drive_back=False, point_cloud_laser_topic_name=None,
                                          clear_path=True,
                                          laser_avoidance_angle_cutout=np.pi/5)

giskard.motion_goals.allow_all_collisions()
giskard.execute()
