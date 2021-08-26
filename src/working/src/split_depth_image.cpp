#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"
#include "vector"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/highgui/highgui.hpp"

#include <stdint.h>

// Global variables
cv_bridge::CvImagePtr cv_ptr_depth (new cv_bridge::CvImage); // Depth image pointer, cv::Mat form
sensor_msgs::ImagePtr msg_imageLower(new sensor_msgs::Image);
sensor_msgs::ImagePtr msg_imageUpper(new sensor_msgs::Image);
bool trigger = false;

void imageCallbackDepth(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr_depth = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imageUpper(480,640,CV_8UC1);
    cv::Mat imageLower(480,640,CV_8UC1);

    for (int i = 0; i < cv_ptr_depth->image.rows; i++)
    {
        for (int j = 0; j < cv_ptr_depth->image.cols; j++)
        {
            imageUpper.at<uchar>(i,j) = static_cast<uchar>(cv_ptr_depth->image.at<unsigned short>(i,j)/256);
            imageLower.at<uchar>(i,j) = static_cast<uchar>(cv_ptr_depth->image.at<unsigned short>(i,j) - (((unsigned short)(cv_ptr_depth->image.at<unsigned short>(i,j)/256))*256));
        }
    }
    msg_imageLower = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageLower).toImageMsg();
    msg_imageUpper = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageUpper).toImageMsg();
    trigger = true;
//    cv::imshow("image", imageUpper);
//    cv::imshow("image_2", imageLower);
//    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber subDepthImage = it.subscribe("/camera/depth/image_rect_raw", 10, imageCallbackDepth);
    image_transport::Publisher pub_low = it.advertise("camera/depth/image_lower", 1);
    image_transport::Publisher pub_up = it.advertise("camera/depth/image_upper", 1);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
	if(trigger){
 	    pub_low.publish(msg_imageLower);
	    pub_up.publish(msg_imageUpper);
	    trigger = false;
	}
	ROS_INFO("Finished a loop");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
