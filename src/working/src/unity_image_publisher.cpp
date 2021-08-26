//
// Created by thanh-long-vu on 28/1/21.
//

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "working/ROSUnityMsg.h"
#include "opencv2/highgui/highgui.hpp"

// Global variables
std::vector<double> color_info_temp;
std::vector<double> depth_info_temp;
std::vector<uint8_t> color_data;
std::vector<uint8_t> depth_upper_data;
std::vector<uint8_t> depth_lower_data;

working::ROSUnityMsg image_data;

void imageCallbackColor(const sensor_msgs::CompressedImageConstPtr& msg)
{
    color_data.clear();
    color_data = msg->data;
}

void imageCallbackDepthUpper(const sensor_msgs::CompressedImageConstPtr& msg)
{
    depth_upper_data.clear();
    depth_upper_data = msg->data;
}

void imageCallbackDepthLower(const sensor_msgs::CompressedImageConstPtr& msg)
{
    depth_lower_data.clear();
    depth_lower_data = msg->data;
}

void imageInfoCallbackColor(const sensor_msgs::CameraInfoPtr& msg)
{
    color_info_temp.clear();
    color_info_temp.resize(4);

    color_info_temp.at(0) = msg->P.at(2);
    color_info_temp.at(1) = msg->P.at(6);
    color_info_temp.at(2) = msg->P.at(0);
    color_info_temp.at(3) = msg->P.at(5);
}

void imageInfoCallbackDepth(const sensor_msgs::CameraInfoPtr& msg)
{
    depth_info_temp.clear();
    depth_info_temp.resize(4);

    depth_info_temp.at(0) = msg->P.at(2);
    depth_info_temp.at(1) = msg->P.at(6);
    depth_info_temp.at(2) = msg->P.at(0);
    depth_info_temp.at(3) = msg->P.at(5);
}

working::ROSUnityMsg generateCustomMsg()
{
    working::ROSUnityMsg image_data;

    image_data.color = color_data;
    image_data.depth_upper = depth_upper_data;
    image_data.depth_lower = depth_lower_data;

    image_data.color_info = color_info_temp;
    image_data.depth_info = depth_info_temp;

    return image_data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unity_publisher");
    ros::NodeHandle n;

    ros::Subscriber subColorImage = n.subscribe("/compressed/color/compressed", 1, imageCallbackColor);
    ros::Subscriber subDepthImageUpper = n.subscribe("/camera/depth/image_upper/compressed", 1, imageCallbackDepthUpper);
    ros::Subscriber subDepthImageLower = n.subscribe("/camera/depth/image_lower/compressed", 1, imageCallbackDepthLower);

    ros::Subscriber subColorImageInfo = n.subscribe("/camera/color/camera_info", 1, imageInfoCallbackColor);
    ros::Subscriber subDepthImageInfo = n.subscribe("/camera/depth/camera_info", 1, imageInfoCallbackDepth);

    ros::Publisher unityPub = n.advertise<working::ROSUnityMsg>("unity_publisher",10);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if(color_data.size() > 0 && depth_upper_data.size() > 0 && depth_lower_data.size() > 0)
                unityPub.publish(generateCustomMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
