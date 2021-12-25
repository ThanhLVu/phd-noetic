#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdio.h>

// Constants
const double QR_SIZE_OUTTER = 0.226;
const double QR_SIZE_INNER = 0.113;
const double QR_SIZE_CODE = 0.07;
const std::string QR_LINK_NAME = "qr_main_link";
// Past tf value containers
geometry_msgs::TransformStamped base_to_qr_transform;
geometry_msgs::TransformStamped prev_base_to_qr_transform;
geometry_msgs::TransformStamped base_to_qr_frame1;
geometry_msgs::TransformStamped base_to_qr_frame2;
// Flag values to prevent unnecessary variable update
bool status1_flag= false;
bool status2_flag= false;
bool base_to_qr_flag = false;
bool depth1_to_qr_flag = false;
bool depth2_to_qr_flag = false;
bool qr_frame_cam1 = false;
bool qr_frame_cam2 = false;

void generateTransformationMatrix(Eigen::Vector3d& translation, Eigen::Quaterniond& rotation, tf2::Transform& matrix)
{
    tf2::Vector3 translation_tf2(translation(0,0), translation(1,0), translation(2,0));
    tf2::Quaternion rotation_tf2(rotation.x(), rotation.y(), rotation.z(), rotation.w());
    
    matrix.setOrigin(translation_tf2);
    matrix.setRotation(rotation_tf2);
}

void qrStatus1Callback(const std_msgs::Int8::ConstPtr& msg)
{ 
    if(!status1_flag && msg->data == 3)
    {
        status1_flag = true;
        ROS_INFO("Visp is tracking QR for cam1");
    }
}

void qrStatus2Callback(const std_msgs::Int8::ConstPtr& msg)
{ 
    if(!status2_flag && msg->data == 3)
    {
        status2_flag = true;
        ROS_INFO("Visp is tracking QR for cam2");
    }
}

void qrPos1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // If visp status is 3 (Tracking QR)
    if(!depth1_to_qr_flag && status1_flag)
    {
        geometry_msgs::Pose depth1_to_qr_pose;

        depth1_to_qr_pose = msg->pose;

        depth1_to_qr_flag = true;
        ROS_INFO("Got cam1_to_qr1 information");

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_broadcast;

        transform_broadcast.header.stamp = ros::Time::now();
        transform_broadcast.header.frame_id = "camera1_depth_optical_frame";
        transform_broadcast.child_frame_id = "visp_qr_frame1";
        transform_broadcast.transform.translation.x = depth1_to_qr_pose.position.x;
        transform_broadcast.transform.translation.y = depth1_to_qr_pose.position.y;
        transform_broadcast.transform.translation.z = depth1_to_qr_pose.position.z;

        transform_broadcast.transform.rotation.x = depth1_to_qr_pose.orientation.x;
        transform_broadcast.transform.rotation.y = depth1_to_qr_pose.orientation.y;
        transform_broadcast.transform.rotation.z = depth1_to_qr_pose.orientation.z;
        transform_broadcast.transform.rotation.w = depth1_to_qr_pose.orientation.w;

        br.sendTransform(transform_broadcast);
    }
}

void qrPos2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    // If visp status is 3 (Tracking QR)
    if(!depth2_to_qr_flag && status2_flag)
    {
        geometry_msgs::Pose depth2_to_qr_pose;

        depth2_to_qr_pose = msg->pose;

        depth2_to_qr_flag = true;
        ROS_INFO("Got cam2_to_qr2 information");

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_broadcast;

        transform_broadcast.header.stamp = ros::Time::now();
        transform_broadcast.header.frame_id = "camera2_depth_optical_frame";
        transform_broadcast.child_frame_id = "visp_qr_frame2";
        transform_broadcast.transform.translation.x = depth2_to_qr_pose.position.x;
        transform_broadcast.transform.translation.y = depth2_to_qr_pose.position.y;
        transform_broadcast.transform.translation.z = depth2_to_qr_pose.position.z;

        transform_broadcast.transform.rotation.x = depth2_to_qr_pose.orientation.x;
        transform_broadcast.transform.rotation.y = depth2_to_qr_pose.orientation.y;
        transform_broadcast.transform.rotation.z = depth2_to_qr_pose.orientation.z;
        transform_broadcast.transform.rotation.w = depth2_to_qr_pose.orientation.w;

        br.sendTransform(transform_broadcast);
    }
}

void broadcastTransform(tf2::Transform& transform, std::string parent_id, std::string child_id)
{
    // Broadcast the transformation
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform_broadcast;

    transform_broadcast.header.stamp = ros::Time::now();
    transform_broadcast.header.frame_id = parent_id;
    transform_broadcast.child_frame_id = child_id;
    transform_broadcast.transform.translation.x = transform.getOrigin().x();
    transform_broadcast.transform.translation.y = transform.getOrigin().y();
    transform_broadcast.transform.translation.z = transform.getOrigin().z();

    transform_broadcast.transform.rotation.x = transform.getRotation().x();
    transform_broadcast.transform.rotation.y = transform.getRotation().y();
    transform_broadcast.transform.rotation.z = transform.getRotation().z();
    transform_broadcast.transform.rotation.w = transform.getRotation().w();

    br.sendTransform(transform_broadcast);
}

void resetFlags()
{
    status1_flag = false;
    status2_flag = false;
    base_to_qr_flag = false;
    depth1_to_qr_flag = false;
    depth2_to_qr_flag = false;
    qr_frame_cam1 = false;
    qr_frame_cam2 = false;
}

void getBaseToQR(tf2::Transform &base_to_qr_matrix)
{
    base_to_qr_flag = true;
    if(qr_frame_cam1 || qr_frame_cam2)
    {
        if(qr_frame_cam1 && !qr_frame_cam2)
        {   
            base_to_qr_transform.header.frame_id = base_to_qr_frame1.header.frame_id;
            base_to_qr_transform.transform = base_to_qr_frame1.transform;
            base_to_qr_transform.child_frame_id = QR_LINK_NAME;
        }
        if(qr_frame_cam2 && !qr_frame_cam1)
        {
            base_to_qr_transform.header.frame_id = base_to_qr_frame2.header.frame_id;
            base_to_qr_transform.transform = base_to_qr_frame2.transform;
            base_to_qr_transform.child_frame_id = QR_LINK_NAME;
        }
        if(qr_frame_cam1 && qr_frame_cam2)
        {
            if(base_to_qr_frame1.header.frame_id == base_to_qr_frame2.header.frame_id)
            {
                base_to_qr_transform = base_to_qr_frame1;
                base_to_qr_transform.child_frame_id = QR_LINK_NAME;
                base_to_qr_transform.transform.translation.x = (base_to_qr_frame1.transform.translation.x 
                                                            + base_to_qr_frame2.transform.translation.x) / 2;
                base_to_qr_transform.transform.translation.y = (base_to_qr_frame1.transform.translation.y 
                                                            + base_to_qr_frame2.transform.translation.y) / 2;
                base_to_qr_transform.transform.translation.z = (base_to_qr_frame1.transform.translation.z 
                                                            + base_to_qr_frame2.transform.translation.z) / 2;
                base_to_qr_transform.transform.rotation.w = (base_to_qr_frame1.transform.rotation.w 
                                                            + base_to_qr_frame2.transform.rotation.w) / 2;
                base_to_qr_transform.transform.rotation.x = (base_to_qr_frame1.transform.rotation.x 
                                                            + base_to_qr_frame2.transform.rotation.x) / 2;
                base_to_qr_transform.transform.rotation.y = (base_to_qr_frame1.transform.rotation.y 
                                                            + base_to_qr_frame2.transform.rotation.y) / 2;
                base_to_qr_transform.transform.rotation.z = (base_to_qr_frame1.transform.rotation.z
                                                            + base_to_qr_frame2.transform.rotation.z) / 2;
            }
            else ROS_ERROR("Wrong frame. Wrong frame. Wrong frame");
        }
        prev_base_to_qr_transform = base_to_qr_transform;
    }
    else base_to_qr_transform = prev_base_to_qr_transform;

    Eigen::Vector3d base_to_qr_trans(base_to_qr_transform.transform.translation.x, base_to_qr_transform.transform.translation.y, base_to_qr_transform.transform.translation.z);
    Eigen::Quaterniond base_to_qr_rot(base_to_qr_transform.transform.rotation.w, base_to_qr_transform.transform.rotation.x, base_to_qr_transform.transform.rotation.y, base_to_qr_transform.transform.rotation.z);
    generateTransformationMatrix(base_to_qr_trans, base_to_qr_rot, base_to_qr_matrix);

    broadcastTransform(base_to_qr_matrix, "base_link", QR_LINK_NAME);
}

void generateQRCorners(tf2::Transform& base_to_qr_matrix, std::vector<tf2::Transform>& corners_list)
{
    corners_list.clear();
    corners_list.resize(4);

    tf2::Transform top_left_matrix; 
    Eigen::Vector3d top_left_trans(QR_SIZE_OUTTER/2, QR_SIZE_OUTTER/2, 0);
    Eigen::Quaterniond top_left_rot(1, 0, 0, 0);
    generateTransformationMatrix(top_left_trans, top_left_rot, top_left_matrix);

    corners_list.at(0) = base_to_qr_matrix * top_left_matrix;

    tf2::Transform top_right_matrix; 
    Eigen::Vector3d top_right_trans(-QR_SIZE_OUTTER/2, QR_SIZE_OUTTER/2, 0);
    Eigen::Quaterniond top_right_rot(1, 0, 0, 0);
    generateTransformationMatrix(top_right_trans, top_right_rot, top_right_matrix);

    corners_list.at(1) = base_to_qr_matrix * top_right_matrix;

    tf2::Transform bot_left_matrix; 
    Eigen::Vector3d bot_left_trans(QR_SIZE_OUTTER/2, -QR_SIZE_OUTTER/2, 0);
    Eigen::Quaterniond bot_left_rot(1, 0, 0, 0);
    generateTransformationMatrix(bot_left_trans, bot_left_rot, bot_left_matrix);

    corners_list.at(2) = base_to_qr_matrix * bot_left_matrix;

    tf2::Transform bot_right_matrix; 
    Eigen::Vector3d bot_right_trans(-QR_SIZE_OUTTER/2, -QR_SIZE_OUTTER/2, 0);
    Eigen::Quaterniond bot_right_rot(1, 0, 0, 0);
    generateTransformationMatrix(bot_right_trans, bot_right_rot, bot_right_matrix);

    corners_list.at(3) = base_to_qr_matrix * bot_right_matrix;
}

void publishMarkers(std::vector<tf2::Transform>& corners_list)
{
    for(int i=0; i<corners_list.size(); i++)
    {
        broadcastTransform(corners_list.at(i), "base_link", "corner_"+i);
    }
    resetFlags();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_camera_calibration_qr");

    // NodeHandle
    ros::NodeHandle nh;

    // Steps before run:
    // + Start all nodes required to have a full tf_tree from base_link to qr_link
    // + Start visp_auto_tracker node
    // + Check the names of frames (in tf_tree and code)
    // Note: Make sure the tf tree has base_link connected to qr_link and 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //  Get qr_tag status and position (from visp_auto_tracker)
    ros::Subscriber sub_qr_pos1 = nh.subscribe("/visp_auto_tracker1/object_position", 1, qrPos1Callback);
    ros::Subscriber sub_qr_status1 = nh.subscribe("/visp_auto_tracker1/status", 1, qrStatus1Callback);

    ros::Subscriber sub_qr_pos2 = nh.subscribe("/visp_auto_tracker2/object_position", 1, qrPos2Callback);
    ros::Subscriber sub_qr_status2 = nh.subscribe("/visp_auto_tracker2/status", 1, qrStatus2Callback);

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        // Get transform from camera_link to depth_link (from the urdf model)
        if(depth1_to_qr_flag && status1_flag)
        {
            try
            {
                // Change link name if needed
                base_to_qr_frame1 = tfBuffer.lookupTransform("visp_qr_frame1", "base_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            
            qr_frame_cam1 = true;
            ROS_INFO("Got qr position from cam 1");
        }

        if(depth2_to_qr_flag && status2_flag)
        {
            try
            {
                // Change link name if needed
                base_to_qr_frame2 = tfBuffer.lookupTransform("visp_qr_frame2", "base_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            qr_frame_cam2 = true;
            ROS_INFO("Got qr position from cam 2");
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
