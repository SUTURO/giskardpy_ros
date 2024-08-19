import typing
from typing import List, Optional

import numpy as np
from geometry_msgs.msg import Quaternion, Point
from py_trees.common import Status
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from line_profiler import profile

import giskardpy.casadi_wrapper as w
from giskardpy.god_map import god_map
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from giskardpy.utils.math import rotation_matrix_from_axis_angle, quaternion_from_rotation_matrix


class DebugMarkerPublisher(GiskardBehavior):
    colors = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # red
              ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # blue
              ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # yellow
              ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),  # violet
              ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),  # cyan
              ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # green
              ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),  # white
              ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # black
              ]

    @profile
    def __init__(self, name: str = 'debug marker', tf_topic: str = '/tf', map_frame: Optional[str] = None):
        super().__init__(name)
        if map_frame is None:
            self.map_frame = str(god_map.world.root_link_name)
        else:
            self.map_frame = map_frame
        self.tf_pub = rospy.node.create_publisher(TFMessage, tf_topic, 10)
        self.marker_pub = rospy.node.create_publisher(MarkerArray, f'{rospy.node.get_name()}/visualization_marker_array', 10)

    def setup(self, **kwargs: typing.Any) -> None:
        self.clear_markers()
        super().setup(**kwargs)

    def publish_debug_markers(self):
        ms = MarkerArray()
        ms.markers.extend(self.to_vectors_markers())
        self.marker_pub.publish(ms)

    def to_vectors_markers(self, width: float = 0.05) -> List[Marker]:
        ms = []
        color_counter = 0
        for name, value in self.debugs_evaluated.items():
            expr = self.debugs[name]
            if not hasattr(expr, 'reference_frame'):
                continue
            if expr.reference_frame is not None:
                map_T_ref = god_map.world.compute_fk_np(god_map.world.root_link_name, expr.reference_frame)
            else:
                map_T_ref = np.eye(4)
            if isinstance(expr, w.TransMatrix):
                ref_T_d = value
                map_T_d = np.dot(map_T_ref, ref_T_d)
                map_P_d = map_T_d[:4, 3:]
                # x
                d_V_x_offset = np.array([width, 0., 0., 0.])
                map_V_x_offset = np.dot(map_T_d, d_V_x_offset)
                mx = Marker()
                mx.action = Marker.ADD
                mx.header.frame_id = self.map_frame
                mx.ns = f'debug{name}'
                mx.id = 0
                mx.type = Marker.CYLINDER
                mx.pose.position.x = map_P_d[0][0] + map_V_x_offset[0]
                mx.pose.position.y = map_P_d[1][0] + map_V_x_offset[1]
                mx.pose.position.z = map_P_d[2][0] + map_V_x_offset[2]
                d_R_x = rotation_matrix_from_axis_angle([0, 1, 0], np.pi / 2)
                map_R_x = np.dot(map_T_d, d_R_x)
                q = quaternion_from_rotation_matrix(map_R_x)
                mx.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                mx.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                mx.scale.x = width / 4
                mx.scale.y = width / 4
                mx.scale.z = width * 2
                ms.append(mx)
                # y
                d_V_y_offset = np.array([0, width, 0, 0])
                map_V_y_offset = np.dot(map_T_d, d_V_y_offset)
                my = Marker()
                my.action = Marker.ADD
                my.header.frame_id = self.map_frame
                my.ns = f'debug{name}'
                my.id = 1
                my.type = Marker.CYLINDER
                my.pose.position.x = map_P_d[0][0] + map_V_y_offset[0]
                my.pose.position.y = map_P_d[1][0] + map_V_y_offset[1]
                my.pose.position.z = map_P_d[2][0] + map_V_y_offset[2]
                d_R_y = rotation_matrix_from_axis_angle([1, 0, 0], -np.pi / 2)
                map_R_y = np.dot(map_T_d, d_R_y)
                q = quaternion_from_rotation_matrix(map_R_y)
                my.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                my.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                my.scale.x = width / 4
                my.scale.y = width / 4
                my.scale.z = width * 2
                ms.append(my)
                # z
                d_V_z_offset = np.array([0., 0, width, 0])
                map_V_z_offset = np.dot(map_T_d, d_V_z_offset)
                mz = Marker()
                mz.action = Marker.ADD
                mz.header.frame_id = self.map_frame
                mz.ns = f'debug{name}'
                mz.id = 2
                mz.type = Marker.CYLINDER
                mz.pose.position.x = map_P_d[0][0] + map_V_z_offset[0]
                mz.pose.position.y = map_P_d[1][0] + map_V_z_offset[1]
                mz.pose.position.z = map_P_d[2][0] + map_V_z_offset[2]
                q = quaternion_from_rotation_matrix(map_T_d)
                mz.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                mz.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
                mz.scale.x = width / 4
                mz.scale.y = width / 4
                mz.scale.z = width * 2
                ms.append(mz)
            else:
                m = Marker()
                m.action = Marker.ADD
                m.ns = f'debug/{name}'
                m.id = 0
                m.header.frame_id = self.map_frame
                m.pose.orientation.w = 1.
                if isinstance(expr, w.Vector3):
                    ref_V_d = value
                    if expr.vis_frame is not None:
                        map_T_vis = god_map.world.compute_fk_np(god_map.world.root_link_name, expr.vis_frame)
                    else:
                        map_T_vis = np.eye(4, dtype=float)
                    map_V_d = np.dot(map_T_ref, ref_V_d)
                    map_P_vis = map_T_vis[:4, 3:].T[0]
                    map_P_p1 = map_P_vis
                    map_P_p2 = map_P_vis + map_V_d * 0.5
                    m.points.append(Point(x=map_P_p1[0], y=map_P_p1[1], z=map_P_p1[2]))
                    m.points.append(Point(x=map_P_p2[0], y=map_P_p2[1], z=map_P_p2[2]))
                    m.type = Marker.ARROW
                    if expr.color is None:
                        m.color = self.colors[color_counter]
                    else:
                        m.color = ColorRGBA(r=expr.color.r, g=expr.color.g, b=expr.color.b, a=expr.color.a)
                    m.scale.x = width / 2
                    m.scale.y = width
                    m.scale.z = 0.
                    color_counter += 1
                elif isinstance(expr, w.Point3):
                    ref_P_d = value
                    map_P_d = np.dot(map_T_ref, ref_P_d)
                    m.pose.position.x = map_P_d[0]
                    m.pose.position.y = map_P_d[1]
                    m.pose.position.z = map_P_d[2]
                    m.pose.orientation.w = 1.
                    m.type = Marker.SPHERE
                    if expr.color is None:
                        m.color = self.colors[color_counter]
                    else:
                        m.color = ColorRGBA(r=expr.color.r, g=expr.color.g, b=expr.color.b, a=expr.color.a)
                    m.scale.x = width
                    m.scale.y = width
                    m.scale.z = width
                    color_counter += 1
                ms.append(m)
        return ms

    def clear_markers(self):
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        self.marker_pub.publish(msg)

    @record_time
    @profile
    def update(self):
        self.debugs = god_map.debug_expression_manager.debug_expressions
        if len(self.debugs) > 0:
            self.debugs_evaluated = god_map.debug_expression_manager.evaluated_debug_expressions
            self.publish_debug_markers()
        return Status.SUCCESS
