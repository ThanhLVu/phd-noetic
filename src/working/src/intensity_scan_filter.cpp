#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>

/*
 * Disclaimer: Currently this code is doing filter by distance as the intensity data is not available
 */

sensor_msgs::LaserScan intensity_filtered_msg_;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    intensity_filtered_msg_ = *msg;
    intensity_filtered_msg_.header.stamp += ros::Duration(0.03);

    std::cout << "Original msg: " << std::endl;

    for(int i=0; i<msg->ranges.size(); i++)
    {
        std::cout << msg->ranges.at(i) << " ";
        if (msg->ranges.at(i) > 29) {
            intensity_filtered_msg_.ranges.at(i) = 0;
        }
    }
    std::cout << "Generated filtered msg: " << std::endl;
    for(int i=0; i<intensity_filtered_msg_.ranges.size(); i++)
    {
        std::cout << intensity_filtered_msg_.ranges.at(i) << " ";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intensity_scan_filter");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("sick_s300/scan_filtered", 1000, laserScanCallback);
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("sick_s300/intensity_scan_filtered", 1000);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        laser_pub.publish(intensity_filtered_msg_);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}