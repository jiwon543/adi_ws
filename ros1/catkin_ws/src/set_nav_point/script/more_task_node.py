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
from pymycobot.genre import Angle

mc = MyCobot("/dev/ttyACM0", 115200)

def target_1():  
    #控制机械臂以30mm/s的速度回到初始位置
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)  
    time.sleep(3)
    mc.send_angles([-3.42, -104.76, -31.28, 46.31, -0.26, 46.23], 50)   #去目标点进行夹取
    time.sleep(3)
    mc.set_gripper_state(1,80)   #夹爪控制.夹爪合拢状态
    time.sleep(3)
    mc.send_angles([83.05, -103.53, 143.87, -72.15, 3.25, 46.49], 50)  #收缩
    time.sleep(5)

def target_2():
    time.sleep(1)
    mc.send_angles([92.02, -39.99, -31.46, 52.73, -2.02, 46.14], 50)  #去到目标点投放
    time.sleep(3)
    mc.set_gripper_state(0,80)   #夹爪控制.夹爪打开

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('simple_navigation_goals')
    # 创建MoveBaseClient实例
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # 等待move_base服务器启动
    client.wait_for_server()
    
    target_1()  

    # 定义第一个目标点的坐标系和位置
    goal1 = MoveBaseGoal()
    goal1.target_pose.header.frame_id = "map" # 设置坐标系为地图坐标系
    goal1.target_pose.header.stamp = rospy.Time.now() # 设置时间戳为当前时间
    goal1.target_pose.pose.position.x = 1.3 # 设置目标点的x坐标
    goal1.target_pose.pose.position.y = -1.06 # 设置目标点的y坐标
    goal1.target_pose.pose.orientation.w = 1.0 # 设置目标点的朝向为正方向

    # 发送第一个目标点
    rospy.loginfo("Sending goal 1")
    client.send_goal(goal1)
 
    # 等待机器人到达第一个目标点
    client.wait_for_result()

    # 输出机器人到达第一个目标点的结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Robot reached goal 1")
        target_2()
        rospy.spin()
    else:
        rospy.loginfo("The robot failed to reach goal 1")

    # # 定义第二个目标点的坐标系和位置
    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "map" # 设置坐标系为地图坐标系
    goal2.target_pose.header.stamp = rospy.Time.now() # 设置时间戳为当前时间
    goal2.target_pose.pose.position.x = 0 # 设置目标点的x坐标
    goal2.target_pose.pose.position.y = 0 # 设置目标点的y坐标
    goal2.target_pose.pose.orientation.w = 1.0 # 设置目标点的朝向为正方向

    # # 发送第二个目标点
    rospy.loginfo("Sending goal 2")
    client.send_goal(goal2)

    # # 等待机器人到达第二个目标点
    client.wait_for_result()

    # # 输出机器人到达第二个目标点的结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Robot reached goal 2")

    else:
        rospy.loginfo("The robot failed to reach goal 2")

