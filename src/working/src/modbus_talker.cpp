#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher gripper_pub = n.advertise<std_msgs::Bool>("/mymodbusslave/gripper/write", 1000);
    ros::Publisher name_pub = n.advertise<std_msgs::Int16>("/mymodbusslave/name/write", 1000);
    ros::Publisher signal_pub = n.advertise<std_msgs::Bool>("/mymodbusslave/signal/write", 1000);

    ros::Rate loop_rate(10);
    
    int option = 0;

    while (ros::ok())
    {
        bool bool_value;
        int int_value;
        std_msgs::Bool bool_msg;
        std_msgs::Int16 int_msg;

        ROS_INFO("Choose a topic to talk to:\n1. Publish gripper\n2. Publish name\n3. Publish signal");
        std::cin >> option;
        switch (option)
        {
        case 1:
            ROS_INFO("Give a value (BOOL): ");
            std::cin >> bool_value;
            bool_msg.data = bool_value;
            gripper_pub.publish(bool_msg);
            break;
        
        case 2:
            ROS_INFO("Give a value (INT16): ");
            std::cin >> int_value;
            int_msg.data = int_value;
            name_pub.publish(int_msg);
            break;

        case 3:
            ROS_INFO("Give a value (BOOL): ");
            std::cin >> bool_value;
            bool_msg.data = bool_value;
            name_pub.publish(bool_msg);
            break;

        default:
            break;
        }

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}