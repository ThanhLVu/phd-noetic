#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include <stdio.h>
#include <unistd.h>

//Publishers
ros::Publisher pub_trans_x, pub_trans_y, pub_trans_z, pub_rot_x, pub_rot_y;

control_msgs::JointControllerState state_trans_x, state_trans_y, state_trans_z, state_rot_x, state_rot_y;
std_msgs::Float64 trans_x, trans_y, trans_z, rot_x, rot_y;

void lidar1TransXCb(const control_msgs::JointControllerState::ConstPtr &msg)
{
    state_trans_x = *msg;
}

void lidar1TransYCb(const control_msgs::JointControllerState::ConstPtr &msg)
{
    state_trans_y = *msg;
}

void lidar1TransZCb(const control_msgs::JointControllerState::ConstPtr &msg)
{
    state_trans_z = *msg;
}

void lidar1RotXCb(const control_msgs::JointControllerState::ConstPtr &msg)
{
    state_rot_x = *msg;
}

void lidar1RotYCb(const control_msgs::JointControllerState::ConstPtr &msg)
{
    state_rot_y = *msg;
}

void translateX(std_msgs::Float64 msg)
{
    pub_trans_x.publish(msg);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();
    while (1)
    {
        std::cout << "Process X:\nTarget:" << state_trans_x.set_point << "\nCurrent:" << state_trans_x.process_value << std::endl;
        if (abs(state_trans_x.process_value_dot) < 0.02)
            break;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void translateY(std_msgs::Float64 msg)
{
    pub_trans_y.publish(msg);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();
    while (1)
    {
        std::cout << "Process Y:\nTarget:" << state_trans_y.set_point << "\nCurrent:" << state_trans_y.process_value << std::endl;
        if (abs(state_trans_y.process_value_dot) < 0.02)
            break;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void translateZ(std_msgs::Float64 msg)
{
    pub_trans_z.publish(msg);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    while (1)
    {
        ros::spinOnce();
        std::cout << "Process Z:\nTarget:" << state_trans_z.set_point << "\nCurrent:" << state_trans_z.process_value << std::endl;
        if (abs(state_trans_z.process_value_dot) < 0.02)
            break;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void rotateX(std_msgs::Float64 msg)
{
    pub_rot_x.publish(msg);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();
    while (1)
    {
        std::cout << "Process RotX:\nTarget:" << state_rot_x.set_point << "\nCurrent:" << state_rot_x.process_value << std::endl;
        if (abs(state_rot_x.process_value_dot) < 0.02)
            break;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void rotateY(std_msgs::Float64 msg)
{
    pub_rot_y.publish(msg);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();
    while (1)
    {
        std::cout << "Process RotY:\nTarget:" << state_rot_y.set_point << "\nCurrent:" << state_rot_y.process_value << std::endl;
        if (abs(state_rot_y.process_value_dot) < 0.02)
            break;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void planLidarMovement(std_msgs::Float64 dest_trans_x, std_msgs::Float64 dest_trans_y, std_msgs::Float64 dest_trans_z, std_msgs::Float64 dest_rot_x, std_msgs::Float64 dest_rot_y, int lidar_id)
{
    ROS_INFO("Planning movement...");
    bool ceiling = false;
    bool wall = false;
    if (state_trans_z.process_value < 1)
    {
        ceiling = true;
        ROS_INFO("Currently on ceiling");
    }
    if (round(state_trans_y.process_value) == 8 || round(state_trans_y.process_value) == -8)
    {
        wall = true;
        ROS_INFO("Currently on wall");
    }

    // Do translation
    ROS_INFO("Start translation");
    translateX(dest_trans_x);

    if (ceiling && !wall)
    {
        translateY(dest_trans_y);
        translateZ(dest_trans_z);
    }
    else if (wall && !ceiling)
    {
        translateZ(dest_trans_z);
        translateY(dest_trans_y);
    }

    else if (wall && ceiling)
    {
        if (dest_trans_z.data != state_trans_z.process_value)
            translateZ;
        else if (dest_trans_y.data != state_trans_y.process_value)
            translateY;
    }

    ROS_INFO("Start rotation");
    // Do rotation
    rotateX(dest_rot_x);
    rotateY(dest_rot_y);
}

void MoveLidar(int lidar_id)
{
    bool repeat = true;
    while (repeat)
    {

        ROS_INFO("Destination:");
        std::cout << "trans_x: ";
        std::cin >> trans_x.data;
        std::cout << "trans_y: ";
        std::cin >> trans_y.data;
        std::cout << "trans_z: ";
        std::cin >> trans_z.data;
        std::cout << "rot_x: ";
        std::cin >> rot_x.data;
        std::cout << "rot_y: ";
        std::cin >> rot_y.data;
        if ((trans_z.data != 0.5 && trans_y.data != 8) && (trans_z.data != 0 && trans_y.data != -8))
        {
            ROS_INFO("Invalid destination. Please try again");
            continue;
        }

        planLidarMovement(trans_x, trans_y, trans_z, rot_x, rot_y, lidar_id);

        int input;
        ROS_INFO("Choose next action:\nPress 1: New movement\nPress 2: Change lidar or exit");
        std::cin >> input;
        switch (input)
        {
        case (1):
            repeat = true;
            break;

        case (2):
            repeat = false;
            break;

        default:
            repeat = false;
            break;
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_control_nextore");

    ros::NodeHandle n;

    // Publishers
    /*
        Note on xyz translation directions: opposite of the normal gazebo axis directions
    */
    pub_trans_x = n.advertise<std_msgs::Float64>("/lidar3D/lidar_1_pris1_joint_position_controller/command", 10);
    pub_trans_y = n.advertise<std_msgs::Float64>("/lidar3D/lidar_1_pris2_joint_position_controller/command", 10);
    pub_trans_z = n.advertise<std_msgs::Float64>("/lidar3D/lidar_1_pris3_joint_position_controller/command", 10);
    /*
        Note on rotation around x,y axis: I have no idea which way is positive/negative
    */
    pub_rot_x = n.advertise<std_msgs::Float64>("/lidar3D/lidar_1_rev1_joint_position_controller/command", 10);
    pub_rot_y = n.advertise<std_msgs::Float64>("/lidar3D/lidar_1_rev2_joint_position_controller/command", 10);

    // Subscribers
    ros::Subscriber sub_trans_x = n.subscribe("/lidar3D/lidar_1_pris1_joint_position_controller/state", 1, lidar1TransXCb);
    ros::Subscriber sub_trans_y = n.subscribe("/lidar3D/lidar_1_pris2_joint_position_controller/state", 1, lidar1TransYCb);
    ros::Subscriber sub_trans_z = n.subscribe("/lidar3D/lidar_1_pris3_joint_position_controller/state", 1, lidar1TransZCb);
    ros::Subscriber sub_rot_x = n.subscribe("/lidar3D/lidar_1_rev1_joint_position_controller/state", 1, lidar1RotXCb);
    ros::Subscriber sub_rot_y = n.subscribe("/lidar3D/lidar_1_rev2_joint_position_controller/state", 1, lidar1RotYCb);

    ros::Rate loop_rate(10);

    bool first_msg = false;
    int count = 0;

    while (ros::ok())
    {
        int input;
        ROS_INFO("Started custom controller\nCurrent sensor position:\nTranslation: x=%f    y=%f    z=%f\nRotation:    x=%f    y=%f",
                 state_trans_x.process_value, state_trans_y.process_value, state_trans_z.process_value, state_rot_x.process_value, state_rot_y.process_value);
        if (count < 5)
            count++;
        else
        {
            ROS_INFO("Options:\nPress 1: Move Lidar 1\nPress 2: Move Lidar 2\nPress 3: Exit");
            std::cin >> input;

            switch (input)
            {
            case (1):
                MoveLidar(1);
                break;
            case (2):
                MoveLidar(2);
                break;
            case (3):
                return 0;
            default:
                ROS_INFO("Please check input again");
                break;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}