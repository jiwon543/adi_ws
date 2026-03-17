#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include<tf/transform_listener.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/String.h>
#include<move_base/move_base.h>

float dis_x = 0;
bool flag_data = false;
bool flag = false;
bool temp1 = false;
bool temp2 = false ;
double maker_dis = 0.3;


ros::Publisher lifter_pub;
ros::Publisher stop_pub;
ros::Publisher goal_pub;
std_msgs::UInt8 num;
geometry_msgs::PoseStamped goal;

void SaveData(const ar_track_alvar_msgs::AlvarMarkersPtr& msg)
{
    for(int i = 0;i<msg->markers.size();i++)
    {
        if(msg->markers[i].id == 0)
        {
            dis_x = msg->markers[i].pose.pose.position.x;
            
            if(0.1 < dis_x && dis_x < maker_dis)
            {
                num.data = 1;
                ROS_INFO("pub num :1");
				ROS_INFO("dis_x:%f",dis_x);
                flag_data = true;
                stop_pub.publish(actionlib_msgs::GoalID());
            }
            else
            {
                flag_data = false;
                num.data = 2;
            }
            
           // ROS_INFO("get data!");

        }
        else
        {
            ROS_INFO("AR no data!");
        }
    }

        lifter_pub.publish(num);
        

}


void callback(const ar_track_alvar_msgs::AlvarMarkersPtr& msg)
{

    /**/if(msg->markers.size() > 0)
    {
        SaveData(msg);

    }
    else
    {
        ROS_INFO("1 no date!");
    }
    
}

void goal_cb(const geometry_msgs::PoseStamped &goal_msg)
{
    goal = goal_msg;
}

void tem_cb(const std_msgs::String &tem_msg)
{
    if(tem_msg.data == "UP")
    {
            temp1 = true ;
            goal_pub.publish(goal);
    }
    if(tem_msg.data == "DOWN")
    {
            temp2 = true ;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lifter");

    ros::NodeHandle nh;

    ros::Subscriber goal_sub = nh.subscribe("/move_base/current_goal",1,&goal_cb);
    ros::Subscriber marker_sub = nh.subscribe("/camera/ar_pose_marker", 1 , &callback);
    ros::Subscriber tem_sub = nh.subscribe("/chatter_temp",1,&tem_cb);

    lifter_pub  = nh.advertise<std_msgs::UInt8>("chatter_updown", 1);
    stop_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);


        if(flag_data)
        {
                ros::Duration(3.0).sleep();
            
                num.data = 2;
                //lifter_pub.publish(num);
                    lifter_pub.publish(num);

                ROS_INFO("pub num :2");

                flag = true ;
           
        }

    ros::spin();

}

