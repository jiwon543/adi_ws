#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#代码功能：小车在起点使得机械臂回到初始位置；然后去到目标点夹取物体；
#        然后收缩机械臂使得机械臂占有足够小的面积；当LIMO去到目标点后，打开夹爪，释放物体。
#        释放完货物后，再次回到起点。
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyACM0", 115200)

def point_1():
    num = 2
    while(num>0):
        num=num-1
        mc.send_angles([-1.49, 115, -153.45, 30, -33.42, 0], 80)
        # 将灯的颜色为[0,0,50]
        mc.set_color(0, 0, 50)
        time.sleep(1)
        # 让机械臂快速到达该位置
        mc.send_angles([-1.66, 50.53, -130.16, 123.66, -2.63, 0], 80)
        # 将灯的颜色为[0,50,0]
        mc.set_color(0, 50, 0)
        time.sleep(1)
        mc.send_angles([-1.66, 50.53, -130.16, 123.66, -2.63, 50], 80)
        time.sleep(3.5)
        mc.send_angles([-1.66, 50.53, -130.16, 123.66, -2.63, -50], 80)
        time.sleep(3.5)

def point_2():
    num=2 
    mc.send_angles([-95.88, -76.81, 107.75, 61.87, 5.44, 88.85], 60)
    time.sleep(2)
    while(num>0):
        num=num-1
        mc.send_angles([-95.88, -76.81, 107.75, 61.87, 5.44, 88.85], 60)
        time.sleep(1.5)
        mc.send_angles([-95.88, -76.81, 107.75, -30.49, 5.44, 88.85], 60)
        time.sleep(1.5)
        mc.send_angles([-95.88, -76.81, 107.75, 61.87, 5.44, 88.85], 60)
        time.sleep(1.5)

def point_3():
    num=2
    mc.send_angles([38.14, -108.01, 123.57, -12.74, 52.91, 1.23], 60)
    while(num>0):
        num=num-1
        mc.send_angles([38.14, -108.01, 123.57, -12.74, 52.91, 1.23], 60)
        time.sleep(3)
        mc.send_angles([-4.04, 125.41, -115.48, -32.43, 88.94, -24.6], 60)
        time.sleep(3)


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('simple_navigation_goals')
    # 创建MoveBaseClient实例
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # 等待move_base服务器启动
    client.wait_for_server()

    # 定义第一个目标点的坐标系和位置
    goal1 = MoveBaseGoal()
    goal1.target_pose.header.frame_id = "map" # 设置坐标系为地图坐标系
    goal1.target_pose.header.stamp = rospy.Time.now() # 设置时间戳为当前时间
    goal1.target_pose.pose.position.x = 1.63 # 设置目标点的x坐标
    goal1.target_pose.pose.position.y = 0.1 # 设置目标点的y坐标
    goal1.target_pose.pose.orientation.x = 0 
    goal1.target_pose.pose.orientation.y = 0 
    goal1.target_pose.pose.orientation.z = -0.0264309 
    goal1.target_pose.pose.orientation.w = 0.999651

    # 定义第二个目标点的坐标系和位置
    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "map" # 设置坐标系为地图坐标系
    goal2.target_pose.header.stamp = rospy.Time.now() # 设置时间戳为当前时间
    goal2.target_pose.pose.position.x = 0.85598 # 设置目标点的x坐标
    goal2.target_pose.pose.position.y = -0.74 # 设置目标点的y坐标

    goal2.target_pose.pose.orientation.x = 0
    goal2.target_pose.pose.orientation.y = 0
    goal2.target_pose.pose.orientation.z = -0.709988
    goal2.target_pose.pose.orientation.w = 0.704214

    # 定义第三个目标点的坐标系和位置
    goal3 = MoveBaseGoal()
    goal3.target_pose.header.frame_id = "map" # 设置坐标系为地图坐标系
    goal3.target_pose.header.stamp = rospy.Time.now() # 设置时间戳为当前时间
    goal3.target_pose.pose.position.x = 0.26 # 设置目标点的x坐标
    goal3.target_pose.pose.position.y = -0.78 # 设置目标点的y坐标
    
    goal3.target_pose.pose.orientation.x = 0 # 设置目标点的朝向为正方向
    goal3.target_pose.pose.orientation.y = 0 # 设置目标点的朝向为正方向
    goal3.target_pose.pose.orientation.z = 0.999895 # 设置目标点的朝向为正方向
    goal3.target_pose.pose.orientation.w = -0.0144663 # 设置目标点的朝向为正方向

    # 定义原点的坐标系和位置
    goal4 = MoveBaseGoal()
    goal4.target_pose.header.frame_id = "map" # 设置坐标系为地图坐标系
    goal4.target_pose.header.stamp = rospy.Time.now() # 设置时间戳为当前时间
    goal4.target_pose.pose.position.x = 0 # 设置目标点的x坐标
    goal4.target_pose.pose.position.y = 0 # 设置目标点的y坐标
    
    goal4.target_pose.pose.orientation.x = 0 # 设置目标点的朝向为正方向
    goal4.target_pose.pose.orientation.y = 0 # 设置目标点的朝向为正方向
    goal4.target_pose.pose.orientation.z = 0 # 设置目标点的朝向为正方向
    goal4.target_pose.pose.orientation.w = 1.0 # 设置目标点的朝向为正方向

    loop_num=99     #设置循环次数
    
    while(loop_num>0):
        loop_num = loop_num - 1
        # 发送第一个目标点
        rospy.loginfo("Sending goal 1")
        client.send_goal(goal1)
    
        # 等待机器人到达第一个目标点
        client.wait_for_result()

        # 输出机器人到达第一个目标点的结果
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot reached goal 1,start execute the point1")
            point_1()
            time.sleep(5)
        else:
            rospy.loginfo("The robot failed to reach goal 1")

        # 发送第二个目标点
        rospy.loginfo("Sending goal 2")
        client.send_goal(goal2)

        # 等待机器人到达第二个目标点
        client.wait_for_result()

        # 输出机器人到达第二个目标点的结果
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot reached goal 2,satrt execute the point2")
            point_2()
            time.sleep(5)
        else:
            rospy.loginfo("The robot failed to reach goal 2")

        # 发送第三个目标点
        rospy.loginfo("Sending goal 3")
        client.send_goal(goal3)

        # 等待机器人到达第三个目标点
        client.wait_for_result()

        # 输出机器人到达第三个目标点的结果
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot reached goal 3,satrt execute the point3")
            point_3()
            time.sleep(5)
        else:
            rospy.loginfo("The robot failed to reach goal 3")

        # 发送第四个目标点
        rospy.loginfo("Sending goal 4")
        client.send_goal(goal4)

        # 等待机器人到达第四个目标点
        client.wait_for_result()

        # 输出机器人到达第四个目标点的结果
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot reached goal 4,satrt execute the point4")
            time.sleep(3)
        else:
            rospy.loginfo("The robot failed to reach goal 4")

