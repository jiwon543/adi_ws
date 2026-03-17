/**
 * @file set_nav_point_node.cpp
 * @author agx-ppn
 * @brief 代码功能：在机器人自动导航中，可以通过Rviz的2D Nav Goal按键给定机器人前往的目标点以及位姿。
 *        以上代码实现了一个简单的导航目标点发送程序。
 *        它使用ROS中的move_base包和actionlib库来发送目标点到move_base节点，从而控制机器人移动到指定的位置。
 * @version 0.1
 * @date 2023-05-15
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// 定义MoveBaseClient类型，用于发送目标点到move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;

    // 创建MoveBaseClient实例
    MoveBaseClient ac("move_base", true);

    // 等待move_base服务器启动
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // 定义第一个目标点的坐标系和位置
    move_base_msgs::MoveBaseGoal goal1;
    goal1.target_pose.header.frame_id = "map"; // 设置坐标系为地图坐标系
    goal1.target_pose.header.stamp = ros::Time::now(); // 设置时间戳为当前时间

    goal1.target_pose.pose.position.x = 1.0; // 设置目标点的x坐标
    goal1.target_pose.pose.position.y = 2.0; // 设置目标点的y坐标
    goal1.target_pose.pose.orientation.w = 1.0; // 设置目标点的朝向为正方向。

    // 发送第一个目标点
    ROS_INFO("Sending goal 1");
    ac.sendGoal(goal1);

    // 等待机器人到达第一个目标点
    ac.waitForResult();

    // 输出机器人到达第一个目标点的结果
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Robot reached goal 1");
    else
        ROS_INFO("The robot failed to reach goal 1");

    // 定义第二个目标点的坐标系和位置
    move_base_msgs::MoveBaseGoal goal2;
    goal2.target_pose.header.frame_id = "map"; // 设置坐标系为地图坐标系
    goal2.target_pose.header.stamp = ros::Time::now(); // 设置时间戳为当前时间

    goal2.target_pose.pose.position.x = 3.0; // 设置目标点的x坐标
    goal2.target_pose.pose.position.y = 4.0; // 设置目标点的y坐标
    goal2.target_pose.pose.orientation.w = 1.0; // 设置目标点的朝向为正方向

    // 发送第二个目标点
    ROS_INFO("Sending goal 2");
    ac.sendGoal(goal2);

    // 等待机器人到达第二个目标点
    ac.waitForResult();

    // 输出机器人到达第二个目标点的结果
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Robot reached goal 2");
    else
        ROS_INFO("The robot failed to reach goal 2");

    return 0;
}
