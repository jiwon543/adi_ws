#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# E-Stop: 장애물 감지 시 정지 테스트 코드
# lidar_perception.py에서 '/lidar/clusters' 구독 -> '/cmd_vel' 발행

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

# =======================
#    Global Variable
# =======================
cmd_pub = None
STOP_DISTANCE = None
DRIVE_SPEED = None

# ======================
#      Core Func.
# ======================
def get_min_distance(points):
    if len(points) == 0:
        return float('inf')
    
    coords = np.array([[p.x, p.y] for p in points])
    return np.linalg.norm(coords, axis = 1).min()

# ======================
#      Call Back
# ======================
def cluster_callback(msg):
    min_dist = get_min_distance(msg.points)

    cmd = Twist()

    if min_dist < STOP_DISTANCE:
        cmd.linear.x = 0.0
        rospy.loginfo(f"STOP! Distance: {min_dist:.2f}m")
    else:
        cmd.linear.x = DRIVE_SPEED
        rospy.loginfo(f"DRIVE.. Distance: {min_dist:.2f}m")

    cmd_pub.publish(cmd)

# ======================
#         Main
# ======================
def main():
    global cmd_pub, STOP_DISTANCE, DRIVE_SPEED
    rospy.init_node('e_stop')

    # Parameter Load from 'config/e_stop.yaml'
    STOP_DISTANCE = rospy.get_param('~stop_distance', 0.5)
    DRIVE_SPEED = rospy.get_param('~drive_speed', 0.3)

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/lidar/clusters', PointCloud, cluster_callback)

    rospy.loginfo(f"E-Stop Started (dist: {STOP_DISTANCE}m, speed: {DRIVE_SPEED}m/s)")
    rospy.spin()

if __name__ == '__main__':
    main()