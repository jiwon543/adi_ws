#include "ros/ros.h"
#include "aruco/marker.h"
#include "aruco_msgs/MarkerArray.h"
#include "geometry_msgs/Twist.h"

double ar_x,ar_y,ar_z;
geometry_msgs::Twist vel;
int markerID_;

void arcb(const aruco_msgs::MarkerArrayConstPtr &msg)
{
    for (int  i = 0; i < msg->markers.size(); i++)
    {
        if(msg->markers[i].id == markerID_)
        {
            ar_x = msg->markers[i].pose.pose.position.x;
            ar_y = msg->markers[i].pose.pose.position.y;
            ar_z = msg->markers[i].pose.pose.position.z;
            ROS_INFO("AR targer 0 Pose:X:%f,Y:%f,Z:%f",ar_x,ar_y,ar_z);
        }

        vel.linear.x = ar_x * 0.5;
        vel.angular.z = ar_z * 1.0;
        ROS_INFO("Publsh velocity command[%f m/s, %f rad/s]",vel.linear.x,vel.angular.z);
    }
    
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "move_to_ar");
    ros::NodeHandle nh;

    ros::Subscriber ar_sub = nh.subscribe("/aruco_marker_publisher/markers", 10, arcb);

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);

    nh.param("markerID",markerID_, 555);

    while (ros::ok)
    {
        cmd_pub.publish(vel);
        ros::spinOnce();
    } 
    return 0;    
    
}