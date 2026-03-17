#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

ros::Publisher img_pub;
ros::Publisher pub_target;

ros::Subscriber sub_image;

bool getimage = false;
double twist_linear_x = 0.0;
double twist_angular_z = 0.0;
sensor_msgs::Image hsv_image;

void imageCB(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR_STREAM("cv_bridge Exception:"<<e.what());
    }
    cv::Mat image = cvImage -> image;
    cv::Mat hsv = image.clone();
    cv::Mat res = image.clone();
    cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(170,170,70),cv::Scalar(180,190,100),res);
    int h = image.rows;
    int w = image.cols;

    cv::Moments M = cv::moments(res);
    if(M.m00 > 0)
    {
        int cx = int (cvRound(M.m10/M.m00));
        int cy = int (cvRound(M.m01/M.m00));
        ROS_INFO("cx: %d cy: %d", cx, cy);
        cv::circle(image, cv::Point(cx, cy), 10, (0, 0, 255));
        cv::imshow("maskr",image);
        int v = cx - w/2;
        twist_linear_x = 0.1;
        twist_angular_z = -float(v) / 300 * 0.4;
        cv::waitKey(1);
    }
    else{
        ROS_INFO("not found line!");
        twist_linear_x = 0;
        twist_angular_z = 0;
        //cmd_pub.publish(twist);
    }
    sensor_msgs::ImagePtr hsv_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    hsv_image = *hsv_image_;
    

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh;

    ros::Subscriber img_sub = nh.subscribe("/camera/rgb/image_raw", 10, imageCB);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/image_hsv",10);

    while(ros::ok()){
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        cmd_pub.publish(twist);
        ros::spinOnce();
        img_pub.publish(hsv_image);
        //ros::Duration(0.5).sleep();
    }
    return 0;


}