#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# Test Code
# Perception: LiDAR Point Clustering

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from sklearn.cluster import DBSCAN

# =======================
#    Global Variable
# =======================
cluster_pub = None

# DBSCAN Parameters
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

# DBSCAN Clustering with Vector Calculation
def cluster_points(points):
    if len(points) < MIN_SAMPLES:
        return []
    
    labels = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit_predict(points)

    # Exclude Noise & Clster Center Point Calculation
    unique_labels = np.unique(labels[labels != -1])
    centroids = np.array([points[labels == l].mean(axis=0) for l in unique_labels])

    return centroids

# Cluster Centroids => Point Cloud Message
def make_pointcloud(centroids):
    pc = PointCloud()
    pc.header.frame_id = "laser"
    pc.header.stamp = rospy.Time.now()
    pc.points = [Point32(x=c[0], y=c[1], z=0) for c in centroids]
    return pc

# ======================
#      Call Back
# ======================
def scan_callback(msg):
    points = polar_to_cartesain(msg.ranges, msg.angle_min, msg.angle_increment)

    if len(points) == 0:
        return
    
    centroids = cluster_points(points)

    if len(centroids) > 0:
        cluster_pub.publish(make_pointcloud(centroids))


# ======================
#         Main
# ======================
def main():
    global cluster_pub

    rospy.init_node('lidar_perception')
    
    cluster_pub = rospy.Publisher('/lidar/clusters', PointCloud, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    
    rospy.loginfo("LiDAR Perception Node Started")
    rospy.spin()

if __name__ == '__main__':
    main()