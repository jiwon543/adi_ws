#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# Test Code
# Perception: LiDAR Point Clustering

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sklearn.cluster import DBSCAN

# =======================
#    Global Variable
# =======================
cluster_pub = None
marker_pub = None

# RViz Colors for Clusters
COLORS = [
    (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    (1.0, 1.0, 0.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0),
]

# DBSCAN Parameters (default, overridden by yaml via rospy.get_param)
EPS = 0.3 # Neighbor (m)
MIN_SAMPLES = 5 # Num. of Minimum Point

# ======================
#      Core Func.
# ======================

# Convert Polar -> Cartesian
def polar_to_cartesain(ranges, angle_min, angle_increment):
    # Vector Calculation using Numpy
    ranges = np.array(ranges)
    angles = angle_min + np.arange(len(ranges)) * angle_increment

    # Filtering Valid Points
    valid = (ranges > 0.01) & np.isfinite(ranges)
    r, a = ranges[valid], angles[valid]

    return np.column_stack((r * np.cos(a), r * np.sin(a)))

# DBSCAN Clustering — returns (points, labels)
def cluster_points(points):
    if len(points) < MIN_SAMPLES:
        return points, np.full(len(points), -1)

    labels = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit_predict(points)
    return points, labels

# Cluster Centroids => PointCloud (for e_stop)
def make_pointcloud(points, labels):
    pc = PointCloud()
    pc.header.frame_id = "laser_link"
    pc.header.stamp = rospy.Time.now()
    unique_labels = np.unique(labels[labels != -1])
    centroids = [points[labels == l].mean(axis=0) for l in unique_labels]
    pc.points = [Point32(x=c[0], y=c[1], z=0) for c in centroids]
    return pc

# Cluster Points => MarkerArray (POINTS type, color per cluster)
def make_markers(points, labels):
    ma = MarkerArray()
    now = rospy.Time.now()
    unique_labels = np.unique(labels[labels != -1])
    for i, l in enumerate(unique_labels):
        cluster_pts = points[labels == l]
        r, g, b = COLORS[i % len(COLORS)]
        m = Marker()
        m.header.frame_id = "laser_link"
        m.header.stamp = now
        m.ns = "clusters"
        m.id = i
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.05  # point size (m)
        m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        m.lifetime = rospy.Duration(0.2)
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in cluster_pts]
        ma.markers.append(m)
    return ma

# ======================
#      Call Back
# ======================
def scan_callback(msg):
    points = polar_to_cartesain(msg.ranges, msg.angle_min, msg.angle_increment)

    if len(points) == 0:
        return
    
    points, labels = cluster_points(points)

    if np.any(labels != -1):
        cluster_pub.publish(make_pointcloud(points, labels))
        marker_pub.publish(make_markers(points, labels))


# ======================
#         Main
# ======================
def main():
    global cluster_pub, marker_pub, EPS, MIN_SAMPLES

    rospy.init_node('lidar_perception')

    EPS = rospy.get_param('~eps', EPS)
    MIN_SAMPLES = int(rospy.get_param('~min_samples', MIN_SAMPLES))

    cluster_pub = rospy.Publisher('/lidar/clusters', PointCloud, queue_size=1)
    marker_pub = rospy.Publisher('/lidar/cluster_markers', MarkerArray, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    
    rospy.loginfo("LiDAR Perception Node Started")
    rospy.spin()

if __name__ == '__main__':
    main()