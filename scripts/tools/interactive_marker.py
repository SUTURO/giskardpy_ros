import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback
import giskardpy_ros.ros2.tfwrapper as tf
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode


class InteractiveMarkerNode:
    def __init__(self, root_link: str, tip_link: str) -> None:
        super().__init__()
        self.root_link = root_link
        self.tip_link = tip_link
        self.giskard = GiskardWrapperNode('interactive_cartesian_goals')
        self.giskard.spin_in_background()
        tf.init(self.giskard.node_handle)

        # Create an interactive marker server
        self.server = InteractiveMarkerServer(self.giskard.node_handle, 'cartesian_goals')

        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.root_link
        int_marker.name = "muh"
        int_marker.scale = 0.25

        # Set the position of the interactive marker
        int_marker.pose = tf.lookup_pose(self.root_link, self.tip_link).pose

        # Create a marker for the interactive marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.175
        box_marker.scale.y = 0.175
        box_marker.scale.z = 0.175
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # Create a control that contains the marker
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        # Add the control to the interactive marker
        int_marker.controls.append(box_control)

        # Create controls to move the marker along all axes
        self.add_control(int_marker, 'move_x', InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0, 1.0)
        self.add_control(int_marker, 'move_y', InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0, 1.0)
        self.add_control(int_marker, 'move_z', InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0, 1.0)

        # Create controls to rotate the marker around all axes
        self.add_control(int_marker, 'rotate_x', InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0, 1.0)
        self.add_control(int_marker, 'rotate_y', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0, 1.0)
        self.add_control(int_marker, 'rotate_z', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0, 1.0)

        # Add the interactive marker to the server
        self.server.insert(int_marker)

        # Set the callback for marker feedback
        self.server.setCallback(int_marker.name, self.process_feedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def add_control(self, int_marker: InteractiveMarker, name: str, interaction_mode: int, x: float, y: float, z: float,
                    w: float) -> None:
        control = InteractiveMarkerControl()
        control.name = name
        control.interaction_mode = interaction_mode
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        int_marker.controls.append(control)

    def process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.giskard.node_handle.get_logger().info(f"Marker feedback received: {feedback.event_type}")
            goal = PoseStamped()
            goal.header = feedback.header
            goal.pose = feedback.pose
            self.giskard.motion_goals.add_cartesian_pose(goal_pose=goal,
                                                         tip_link=self.tip_link,
                                                         root_link=self.root_link)
            self.giskard.motion_goals.allow_all_collisions()
            self.giskard.add_default_end_motion_conditions()
            self.giskard.execute_async()


def main(args: None = None) -> None:
    rclpy.init(args=args)
    node = InteractiveMarkerNode(root_link='world',
                                 tip_link='tool_link')
    node.giskard.node_handle.get_logger().info('interactive marker server running')
    node.giskard.spinner.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
