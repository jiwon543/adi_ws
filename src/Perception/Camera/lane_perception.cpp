// lane_perception.cpp
// Perception Camera node: Sub. camera image -> Lane detection -> Pub. Lane detection topic

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

// ------ 전역 상태 -------
ros::Subsriber g_image_sub;
ros::Publisher g_lane_center_pub;
ros::Publisher g_curvature_pub;
// ------ 파라미터 --------
double g_lane_bev_x = 320.0;
