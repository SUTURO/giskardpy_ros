import string
from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker

import giskardpy.casadi_wrapper as cas
from giskardpy.monitors.payload_monitors import PayloadMonitor


class LidarPayloadMonitor(PayloadMonitor):

    def __init__(self,
                 topic: string = 'hsrb/base_scan',
                 name: Optional[str] = None,
                 frame_id: Optional[str] = 'base_range_sensor_link',
                 laser_distance_threshold_width: Optional[float] = 0.8,
                 laser_distance_threshold: Optional[float] = 0.5,
                 start_condition: cas.Expression = cas.TrueSymbol):
        super().__init__(name=name, stay_true=False, start_condition=start_condition, run_call_in_thread=False)
        self.topic = topic
        self.data = LaserScan()
        self.laser_scan_analyzer = LaserScanThreshold(laser_frame=frame_id,
                                                      laser_scan_topic=topic,
                                                      laser_distance_threshold_width=laser_distance_threshold_width,
                                                      laser_distance_threshold=laser_distance_threshold)

    def __call__(self):
        self.state = self.laser_scan_analyzer.check_collision()


class LaserScanThreshold:
    thresholds: np.ndarray = None
    threshold_publisher: rospy.Publisher = None
    laser_subscriber: rospy.Subscriber = None
    scan: LaserScan = None

    def __init__(self,
                 laser_avoidance_angle_cutout: float = np.pi / 4,
                 laser_distance_threshold_width: float = 0.8,
                 laser_distance_threshold: float = 0.5,
                 laser_frame: str = 'base_range_sensor_link',
                 laser_scan_topic: str = '/hsrb/base_scan',
                 threshold_publisher_topic: str = '~visualization_marker_array'):
        """
        Initializes the Parameters and Publishers/Subscribers for the LidarThresholdAnalyser class
        :param laser_avoidance_angle_cutout: Angle for the cutout of the Threshold depicting the front of the laser scan
        :param laser_distance_threshold_width: Width of the sides of the Threshold of Collision detection
        :param laser_distance_threshold: Upper bound for the Threshold
        :param laser_frame: Name of the Frame from which the LaserScan is published from
        :param laser_scan_topic: Topic on which the LaserScan is published on
        :param threshold_publisher_topic: Topic on which the Thresholds will be published
        """
        self.laser_scan_topic = laser_scan_topic
        self.threshold_publisher_topic = threshold_publisher_topic

        self.laser_avoidance_angle_cutout = laser_avoidance_angle_cutout
        self.laser_distance_threshold_width = laser_distance_threshold_width
        self.laser_distance_threshold = laser_distance_threshold
        self.laser_frame = laser_frame

        self.closest_laser_left = self.laser_distance_threshold_width
        self.closest_laser_right = -self.laser_distance_threshold_width
        self.closest_laser_reading = 0

        self.threshold_publisher = rospy.Publisher(self.threshold_publisher_topic, MarkerArray, queue_size=10,
                                                   latch=True)
        self.laser_subscriber = rospy.Subscriber(self.laser_scan_topic, LaserScan, self._laser_cb, queue_size=10)

    def _calculate_laser_thresholds(self):
        """
        Calculate the Thresholds for the Collision Detection of the Robot
        :return: The Thresholds of the Collision Detection
        """
        thresholds = []
        if len(self.scan.ranges) % 2 == 0:
            print('laser range is even')
            angles = np.arange(self.scan.angle_min,
                               self.scan.angle_max,
                               self.scan.angle_increment)[:-1]
        else:
            angles = np.arange(self.scan.angle_min,
                               self.scan.angle_max,
                               self.scan.angle_increment)
        for angle in angles:
            # Left side of the Thresholds
            if angle < 0:
                y = -self.laser_distance_threshold_width
                length = y / np.sin(angle)
                x = np.cos(angle) * length
                thresholds.append((x, y, length, angle))
            # Rights side of the Thresholds
            else:
                y = self.laser_distance_threshold_width
                length = y / np.sin(angle)
                x = np.cos(angle) * length
                thresholds.append((x, y, length, angle))
            # Check if the upper bound of threshold has been breached (if yes, replace with upper bound)
            if length > self.laser_distance_threshold:
                length = self.laser_distance_threshold
                x = np.cos(angle) * length
                y = np.sin(angle) * length
                thresholds[-1] = (x, y, length, angle)
        thresholds = np.array(thresholds)
        return thresholds

    def _laser_cb(self, data):
        """
        Callback function for Laser sensor data. Initializes the thresholds if there are none and calculates closets
        points to the LaserScanner depending on the thresholds
        """
        self.scan = data
        if self.thresholds is None:
            self.thresholds = self._calculate_laser_thresholds()
            self._publish_laser_thresholds()
        self.closest_laser_reading, self.closest_laser_left, self.closest_laser_right \
            = self._calculate_closest_points_in_threshold()

    def _publish_laser_thresholds(self):
        """
        Publishes the LaserScan-Thresholds as visualisation-marker-array on the topic visualisation_marker_array.
        Includes both the general range of the threshold and the cutout for separating the front and sides of the
         thresholds.
        """

        # Creates LaserScan-Threshold-Line, defined by the self.threshold variable
        ms = MarkerArray()
        m_line = Marker()
        m_line.action = m_line.ADD
        m_line.ns = 'laser_thresholds'
        m_line.id = 1332
        m_line.type = m_line.LINE_STRIP
        m_line.header.frame_id = self.laser_frame
        m_line.scale.x = 0.05
        m_line.color.a = 1
        m_line.color.r = 0.5
        m_line.color.b = 1
        m_line.frame_locked = True
        for item in self.thresholds:
            p = Point()
            p.x = item[0]
            p.y = item[1]
            m_line.points.append(p)
        ms.markers.append(m_line)

        # Create the LaserScan-Threshold-Cutout for separating front and sides
        square = Marker()
        square.action = m_line.ADD
        square.ns = 'laser_avoidance_angle_cutout'
        square.id = 1333
        square.type = m_line.LINE_STRIP
        square.header.frame_id = self.laser_frame
        p = Point()
        idx = np.where(self.thresholds[:, -1] < -self.laser_avoidance_angle_cutout)[0][-1]
        p.x = self.thresholds[idx, 0]
        p.y = self.thresholds[idx, 1]
        square.points.append(p)
        p = Point()
        square.points.append(p)
        p = Point()
        idx = np.where(self.thresholds[:, -1] > self.laser_avoidance_angle_cutout)[0][0]
        p.x = self.thresholds[idx, 0]
        p.y = self.thresholds[idx, 1]
        square.points.append(p)
        square.scale.x = 0.05
        square.color.a = 1
        square.color.r = 0.5
        square.color.b = 1
        square.frame_locked = True
        ms.markers.append(square)

        self.threshold_publisher.publish(ms)

    def clean_up(self):
        """
        Clean up the Laser-Subscriber
        """
        if self.laser_subscriber is not None:
            self.laser_subscriber.unregister()
            self.laser_subscriber = None

    def _calculate_closest_points_in_threshold(self):
        """
        Calculates the closest points within the threshold. Differentiates between the front and the sides of the scan.
        :return: the closest points within the threshold. Divided into the closest front, left and right points.
        """
        data = np.array(self.scan.ranges)
        xs = np.cos(self.thresholds[:, 3]) * data
        ys = np.sin(self.thresholds[:, 3]) * data
        violations = data < self.thresholds[:, 2]
        xs_error = xs - self.thresholds[:, 0]
        half = int(data.shape[0] / 2)
        x_below_laser_avoidance_threshold1 = self.thresholds[:, -1] > self.laser_avoidance_angle_cutout
        x_below_laser_avoidance_threshold2 = self.thresholds[:, -1] < -self.laser_avoidance_angle_cutout
        x_below_laser_avoidance_threshold = x_below_laser_avoidance_threshold1 | x_below_laser_avoidance_threshold2
        y_filter = x_below_laser_avoidance_threshold & violations
        closest_laser_right = ys[:half][y_filter[:half]]
        closest_laser_left = ys[half:][y_filter[half:]]

        x_positive = np.where(np.invert(x_below_laser_avoidance_threshold))[0]
        x_start = x_positive[0]
        x_end = x_positive[-1]

        front_violation = xs_error[x_start:x_end][violations[x_start:x_end]]
        if len(closest_laser_left) > 0:
            closest_laser_left = min(closest_laser_left)
        else:
            closest_laser_left = self.laser_distance_threshold_width
        if len(closest_laser_right) > 0:
            closest_laser_right = max(closest_laser_right)
        else:
            closest_laser_right = -self.laser_distance_threshold_width
        if len(front_violation) > 0:
            closest_laser_reading = min(front_violation)
        else:
            closest_laser_reading = 0
        return closest_laser_reading, closest_laser_left, closest_laser_right

    def check_collision(self):
        """
        Checks if Collision is detected
        :return: True if collision detected, False otherwise
        """

        if (self.laser_distance_threshold_width > self.closest_laser_left
                or self.laser_distance_threshold_width < self.closest_laser_right
                or self.closest_laser_reading != 0):
            return True
        else:
            return False
