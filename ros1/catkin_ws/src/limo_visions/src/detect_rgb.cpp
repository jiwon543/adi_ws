#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>


void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

        // 在OpenCV中进行颜色识别
        cv::Mat image = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);  // 转换为HSV颜色空间

        // 定义颜色的阈值范围
        cv::Scalar lower_bound(30, 50, 50);  // 低阈值（颜色下限）
        cv::Scalar upper_bound(60, 255, 255);  // 高阈值（颜色上限）

        // 创建一个掩码，将在指定颜色范围内的像素设为白色，其他像素设为黑色
        cv::Mat mask;
        cv::inRange(hsv_image, lower_bound, upper_bound, mask);

        // 查找颜色区域的轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 在原图上绘制矩形框出颜色块
        for (const auto& contour : contours)
        {
            cv::Rect bounding_box = cv::boundingRect(contour);
            cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2);  // 用红色矩形框出
        }

        // 在这里，你可以发布处理后的图像或执行其他操作

        // 显示图像
        cv::imshow("Color Detection", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detection_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    ros::spin();

    return 0;
}
