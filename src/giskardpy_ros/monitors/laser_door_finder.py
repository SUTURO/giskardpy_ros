# from typing import Optional, List

# import numpy as np
# import rospy
# from geometry_msgs.msg import Vector3Stamped, PointStamped
#
# import giskardpy.casadi_wrapper as cas
# from giskard_msgs.msg import PointArray
# from giskardpy.casadi_wrapper import Point3
# from giskardpy.data_types.data_types import PrefixName, ObservationState
# from giskardpy.god_map import god_map
# from giskardpy.motion_statechart.monitors.monitors import PayloadMonitor
# from giskardpy_ros.ros1 import msg_converter
#
#
# class LaserDoorFinder(PayloadMonitor):
#     vector: cas.Vector3 = None
#     points: List[Point3] = None
#     door_hinge_name: PrefixName
#     door_length: float
#
#     def __init__(self,
#                  vector_topic: str,
#                  point_topic: str,
#                  door_hinge_name: PrefixName,
#                  door_handle_name: PrefixName,
#                  door_main_name: PrefixName,
#                  name: Optional[str] = None):
#         if name is None:
#             name = self.__class__.__name__
#         super().__init__(run_call_in_thread=False, name=name)
#
#         link_collision = god_map.world.get_link(link_name=door_main_name).collisions[0]
#         self.door_length = sorted([link_collision.height, link_collision.width, link_collision.depth])[-2]
#
#         with god_map.world.reset_joint_state_context():
#             god_map.world.state[door_hinge_name].position = 0
#             self.hinge_closed_V_handle = cas.Vector3(god_map.world.compute_fk_point(door_hinge_name, door_handle_name))
#
#         door_hinge_id = god_map.world.get_movable_parent_joint(door_main_name)
#
#         self.door_hinge_joint = door_hinge_id
#         self.door_hinge_name = door_hinge_name
#
#         self.vec_sub = rospy.Subscriber(name=vector_topic, data_class=Vector3Stamped, queue_size=10, callback=self.vcb)
#         self.point_sub = rospy.Subscriber(name=point_topic, data_class=PointArray, queue_size=10, callback=self.pcb)
#
#     def __call__(self):
#         if self.vector is None or self.points is None:
#             self.state = ObservationState.false
#             return
#
#         far_point = Point3().from_xyz(0, 0, 0)
#         for point in self.points:
#             transform_point = god_map.world.transform(self.door_hinge_name, point)
#             if np.linalg.norm(transform_point.to_np()) > np.linalg.norm(far_point.to_np()):
#                 far_point = transform_point
#
#         self.vector.scale(1)
#         # door_hinge_point = far_point + (self.vector * self.door_length)
#
#         angle = cas.angle_between_vector(self.vector, self.hinge_closed_V_handle).to_np()
#
#         god_map.world.state[self.door_hinge_joint].position = -angle
#
#         self.state = ObservationState.true
#
#     def vcb(self, data: Vector3Stamped):
#         vector = msg_converter.ros_msg_to_giskard_obj(data, god_map.world)
#         self.vector = god_map.world.transform(self.door_hinge_name, vector)
#
#     def pcb(self, data: PointArray):
#         points = []
#         for point in data.points:
#             msg_point = PointStamped()
#             msg_point.header.frame_id = data.header.frame_id
#             msg_point.point = point
#             conv_point = msg_converter.ros_msg_to_giskard_obj(msg_point, god_map.world)
#             points.append(conv_point)
#         self.points = points
