#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3Stamped
from sensor_msgs.msg import LaserScan
from sklearn.linear_model import RANSACRegressor, LinearRegression
from visualization_msgs.msg import Marker

from giskard_msgs.msg import PointArray

debug_mode: bool = False
ransac: bool = True


def process_scan(data):
    # Define the segment of interest (e.g., angles between -30° to +30°)
    angle_min = np.deg2rad(-30)  # -30 degrees
    angle_max = np.deg2rad(30)  # +30 degrees
    ranges = np.array(data.ranges)
    angles = np.linspace(data.angle_min, data.angle_max, len(ranges))

    # Select the segment
    segment_mask = (angles >= angle_min) & (angles <= angle_max)
    segment_ranges = ranges[segment_mask]
    segment_angles = angles[segment_mask]

    # Filter out invalid ranges
    valid_mask = np.isfinite(segment_ranges) & (segment_ranges > 0)
    segment_ranges = segment_ranges[valid_mask]
    segment_angles = segment_angles[valid_mask]

    # Convert polar coordinates to Cartesian coordinates
    x = segment_ranges * np.cos(segment_angles)
    y = segment_ranges * np.sin(segment_angles)

    # Visualize the filtered points
    if debug_mode:
        visualize_points(x, y)

    # Perform linear regression if enough points are valid
    if len(x) > 1 and not ransac:
        slope, intercept = linear_regression_from_points(x, y)
        if debug_mode:
            rospy.loginfo(f"Linear Regression: slope={slope:.2f}, intercept={intercept:.2f}")
        publish_vector(x, y, slope, intercept)
        publish_line(x, y, slope, intercept)
    elif not ransac:
        rospy.logwarn("Not enough points to perform linear regression.")

    if len(x) > 1 and ransac:
        slope, intercept, inlier_mask = fit_line_ransac(x, y)
        if debug_mode:
            rospy.loginfo(f"Linear Regression: slope={slope:.2f}, intercept={intercept:.2f}")
        publish_vector(x[inlier_mask], y[inlier_mask], slope, intercept)
        publish_line(x[inlier_mask], y[inlier_mask], slope, intercept)
    elif ransac:
        rospy.logwarn("Not enough points to perform linear regression.")


def linear_regression_from_points(x, y):
    """
    Perform linear regression on 2D points (x, y).
    :param x: Array of x-coordinates.
    :param y: Array of y-coordinates.
    :return: Slope (m) and intercept (b) of the best-fit line.
    """
    if len(x) != len(y):
        raise ValueError("Input arrays x and y must have the same length.")

    # Ensure inputs are numpy arrays
    x = np.array(x)
    y = np.array(y)

    # Compute slope (m) and intercept (b) using least squares
    A = np.vstack([x, np.ones(len(x))]).T  # Design matrix
    m, b = np.linalg.lstsq(A, y, rcond=None)[0]  # Solve for m, b

    return m, b


def fit_line_ransac(x, y):
    x = x.reshape(-1, 1)  # Reshape for sklearn
    ransac = RANSACRegressor(estimator=LinearRegression(), min_samples=2, residual_threshold=0.05)
    ransac.fit(x, y)
    inlier_mask = ransac.inlier_mask_  # Boolean mask of inliers
    slope = ransac.estimator_.coef_[0]
    intercept = ransac.estimator_.intercept_
    return slope, intercept, inlier_mask


def visualize_points(x, y):
    """
    Visualize the points used for regression as markers in RViz.
    :param x: Array of x-coordinates.
    :param y: Array of y-coordinates.
    """
    marker = Marker()
    marker.header.frame_id = "base_range_sensor_link"
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # Point size
    marker.scale.y = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0  # Blue points

    # Add points to the marker
    for xi, yi in zip(x, y):
        p = Point(x=xi, y=yi, z=0)
        marker.points.append(p)

    marker_points_pub.publish(marker)


def publish_line(x, y, slope, intercept):
    """
    Publish the fitted line as a marker in RViz.
    :param x: Array of x-coordinates.
    :param y: Array of y-coordinates.
    :param slope: Slope of the fitted line.
    :param intercept: Intercept of the fitted line.
    """

    points = PointArray()
    points.header.frame_id = "base_range_sensor_link"

    for xi, yi in zip(x, y):
        p = Point(x=xi, y=yi, z=0)
        points.points.append(p)

    points_pub.publish(points)

    if debug_mode:
        marker = Marker()
        marker.header.frame_id = "base_range_sensor_link"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green line
        marker.color.b = 0.0

        # Define two points on the fitted line
        x_min, x_max = min(x), max(x)
        y_min = slope * x_min + intercept
        y_max = slope * x_max + intercept

        # Add the two points to the marker
        p1 = Point(x=x_min, y=y_min, z=0)
        p2 = Point(x=x_max, y=y_max, z=0)
        marker.points.append(p1)
        marker.points.append(p2)

        line_pub.publish(marker)


def publish_vector(x, y, slope, intercept):
    # Define two points on the fitted line
    x_min, x_max = min(x), max(x)
    y_min = slope * x_min + intercept
    y_max = slope * x_max + intercept

    vect = Vector3Stamped()
    vect.header.frame_id = 'base_range_sensor_link'
    vect.header.stamp = rospy.Time.now()

    vect.vector.x = x_max - x_min
    vect.vector.y = y_max - y_min
    vect.vector.z = 0

    vector_pub.publish(vect)

    if debug_mode:
        marker = Marker()
        marker.header.frame_id = 'base_range_sensor_link'
        marker.header.stamp = rospy.Time.now()

        marker.ns = "vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = [
            Point(x=x_min, y=y_min, z=0),  # Start of the arrow
            Point(x_min + vect.vector.x, y_min + vect.vector.y, vect.vector.z)  # End of the arrow
        ]

        # Arrow appearance
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1  # Head diameter
        marker.scale.z = 0.1  # Head length
        marker.color.a = 1.0  # Transparency
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        vector_marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("line_regression_with_preprocessing_node")

    # Subscribe to the laser scan topic
    rospy.Subscriber("/hsrb/base_scan", LaserScan, process_scan)

    # Publishers for visualizing the points and fitted line
    marker_points_pub = rospy.Publisher("/segment_points_marker", Marker, queue_size=10)
    line_pub = rospy.Publisher("/fitted_line_marker", Marker, queue_size=10)
    vector_marker_pub = rospy.Publisher('/vector_marker', Marker, queue_size=10)
    vector_pub = rospy.Publisher('/door_vector', Vector3Stamped, queue_size=10)
    points_pub = rospy.Publisher("/door_points", PointArray, queue_size=10)

    rospy.spin()
