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

// Past tf value containers
std::vector<Eigen::Vector3d> translation_vec;
std::vector<Eigen::Vector3d> rotation_vec;
geometry_msgs::TransformStamped base_to_qr_transform;
geometry_msgs::Pose depth1_to_qr_pose;
geometry_msgs::TransformStamped cam1_to_depth1_transform;
geometry_msgs::Pose depth2_to_qr_pose;
geometry_msgs::TransformStamped cam2_to_depth2_transform;
// Flag values to prevent unnecessary variable update
bool status1_flag= false;
bool status2_flag= false;
bool base_to_qr_flag = false;
bool depth1_to_qr_flag = false;
bool depth2_to_qr_flag = false;
bool cam1_to_depth1_flag = false;
bool cam2_to_depth2_flag = false;
bool cam1_done = false;
bool cam2_done = false;

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
    cam1_to_depth1_flag = false;
    cam2_to_depth2_flag = false;
}

void calculateTransformationMatrix()
{
    ROS_INFO("Calibrate cam 1");
    if (base_to_qr_flag && depth1_to_qr_flag && cam1_to_depth1_flag)
    {
        // Process base_to_qr transform from tf_tree
        tf2::Transform base_to_qr_matrix;

        Eigen::Vector3d base_to_qr_trans(base_to_qr_transform.transform.translation.x, base_to_qr_transform.transform.translation.y, base_to_qr_transform.transform.translation.z);
        Eigen::Quaterniond base_to_qr_rot(base_to_qr_transform.transform.rotation.w, base_to_qr_transform.transform.rotation.x, base_to_qr_transform.transform.rotation.y, base_to_qr_transform.transform.rotation.z);

        // std::cout << base_to_qr_trans << std::endl;
        // std::cout << base_to_qr_rot.w() << " " << base_to_qr_rot.x() << " " << base_to_qr_rot.y() << " " << base_to_qr_rot.z() << std::endl;

        generateTransformationMatrix(base_to_qr_trans, base_to_qr_rot, base_to_qr_matrix);

        // Process depth_to_qr transform from visp_auto_tracker/object_position topic
        tf2::Transform depth1_to_qr_matrix;

        Eigen::Vector3d depth1_to_qr_trans(depth1_to_qr_pose.position.x, depth1_to_qr_pose.position.y, depth1_to_qr_pose.position.z);
        Eigen::Quaterniond depth1_to_qr_rot(depth1_to_qr_pose.orientation.w, depth1_to_qr_pose.orientation.x, depth1_to_qr_pose.orientation.y, depth1_to_qr_pose.orientation.z);

        generateTransformationMatrix(depth1_to_qr_trans, depth1_to_qr_rot, depth1_to_qr_matrix);

        // std::cout << depth1_to_qr_trans << std::endl;
        // std::cout << depth1_to_qr_rot.w() << " " << depth1_to_qr_rot.x() << " " << depth1_to_qr_rot.y() << " " << depth1_to_qr_rot.z() << std::endl;

        // Process cam_to_depth transform from tf_tree
        tf2::Transform cam1_to_depth1_matrix;

        Eigen::Vector3d cam1_to_depth1_trans(cam1_to_depth1_transform.transform.translation.x, cam1_to_depth1_transform.transform.translation.y, cam1_to_depth1_transform.transform.translation.z);
        Eigen::Quaterniond cam1_to_depth1_rot(cam1_to_depth1_transform.transform.rotation.w, cam1_to_depth1_transform.transform.rotation.x, cam1_to_depth1_transform.transform.rotation.y, cam1_to_depth1_transform.transform.rotation.z);

        // std::cout << cam1_to_depth1_trans << std::endl;
        // std::cout << cam1_to_depth1_rot.w() << " " << cam1_to_depth1_rot.x() << " " << cam1_to_depth1_rot.y() << " " << cam1_to_depth1_rot.z() << std::endl;

        generateTransformationMatrix(cam1_to_depth1_trans, cam1_to_depth1_rot, cam1_to_depth1_matrix);

        // Calculate base_to_cam transform
        tf2::Transform base_to_cam1_matrix;
        base_to_cam1_matrix = base_to_qr_matrix.inverse() * depth1_to_qr_matrix.inverse() * cam1_to_depth1_matrix;

        ROS_INFO("Camera1 to base_link transform: ");
        std::cout << "==> Translation: " << base_to_cam1_matrix.getOrigin().x() << " " << base_to_cam1_matrix.getOrigin().y() << " " << base_to_cam1_matrix.getOrigin().z() << std::endl;
        std::cout << "==> Rotataion quaternion: " << base_to_cam1_matrix.getRotation().w() << " " << base_to_cam1_matrix.getRotation().x() << " " << base_to_cam1_matrix.getRotation().y() << " " << base_to_cam1_matrix.getRotation().z() << std::endl;

        broadcastTransform(base_to_cam1_matrix, "base_link", "camera1_link");
        cam1_done = true;
    }

    ROS_INFO("Calibration cam 2");

    if (base_to_qr_flag && depth2_to_qr_flag && cam2_to_depth2_flag)
    {
        // Process base_to_qr transform from tf_tree
        tf2::Transform base_to_qr_matrix;

        Eigen::Vector3d base_to_qr_trans(base_to_qr_transform.transform.translation.x, base_to_qr_transform.transform.translation.y, base_to_qr_transform.transform.translation.z);
        Eigen::Quaterniond base_to_qr_rot(base_to_qr_transform.transform.rotation.w, base_to_qr_transform.transform.rotation.x, base_to_qr_transform.transform.rotation.y, base_to_qr_transform.transform.rotation.z);

        // std::cout << base_to_qr_trans << std::endl;
        // std::cout << base_to_qr_rot.w() << " " << base_to_qr_rot.x() << " " << base_to_qr_rot.y() << " " << base_to_qr_rot.z() << std::endl;

        generateTransformationMatrix(base_to_qr_trans, base_to_qr_rot, base_to_qr_matrix);

        // Process depth_to_qr transform from visp_auto_tracker/object_position topic
        tf2::Transform depth2_to_qr_matrix;

        Eigen::Vector3d depth2_to_qr_trans(depth2_to_qr_pose.position.x, depth2_to_qr_pose.position.y, depth2_to_qr_pose.position.z);
        Eigen::Quaterniond depth2_to_qr_rot(depth2_to_qr_pose.orientation.w, depth2_to_qr_pose.orientation.x, depth2_to_qr_pose.orientation.y, depth2_to_qr_pose.orientation.z);

        generateTransformationMatrix(depth2_to_qr_trans, depth2_to_qr_rot, depth2_to_qr_matrix);

        // std::cout << depth2_to_qr_trans << std::endl;
        // std::cout << depth2_to_qr_rot.w() << " " << depth2_to_qr_rot.x() << " " << depth2_to_qr_rot.y() << " " << depth2_to_qr_rot.z() << std::endl;

        // Process cam_to_depth transform from tf_tree
        tf2::Transform cam2_to_depth2_matrix;

        Eigen::Vector3d cam2_to_depth2_trans(cam2_to_depth2_transform.transform.translation.x, cam2_to_depth2_transform.transform.translation.y, cam2_to_depth2_transform.transform.translation.z);
        Eigen::Quaterniond cam2_to_depth2_rot(cam2_to_depth2_transform.transform.rotation.w, cam2_to_depth2_transform.transform.rotation.x, cam2_to_depth2_transform.transform.rotation.y, cam2_to_depth2_transform.transform.rotation.z);

        // std::cout << cam2_to_depth2_trans << std::endl;
        // std::cout << cam2_to_depth2_rot.w() << " " << cam2_to_depth2_rot.x() << " " << cam2_to_depth2_rot.y() << " " << cam2_to_depth2_rot.z() << std::endl;

        generateTransformationMatrix(cam2_to_depth2_trans, cam2_to_depth2_rot, cam2_to_depth2_matrix);

        // Calculate base_to_cam transform
        tf2::Transform base_to_cam2_matrix;
        base_to_cam2_matrix = base_to_qr_matrix.inverse() * depth2_to_qr_matrix.inverse() * cam2_to_depth2_matrix;

        ROS_INFO("Camera2 to base_link transform: ");
        std::cout << "==> Translation: " << base_to_cam2_matrix.getOrigin().x() << " " << base_to_cam2_matrix.getOrigin().y() << " " << base_to_cam2_matrix.getOrigin().z() << std::endl;
        std::cout << "==> Rotataion quaternion: " << base_to_cam2_matrix.getRotation().w() << " " << base_to_cam2_matrix.getRotation().x() << " " << base_to_cam2_matrix.getRotation().y() << " " << base_to_cam2_matrix.getRotation().z() << std::endl;

        broadcastTransform(base_to_cam2_matrix, "base_link", "camera2_link");
        resetFlags();
        cam2_done = true;
    }
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

    translation_vec.clear();
    rotation_vec.clear();

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        // Get transform from base_link to qr_link (from the urdf model)
        if(!base_to_qr_flag)
        {
            try
            {
                // Change link name if needed
                base_to_qr_transform = tfBuffer.lookupTransform("qr_link", "base_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            base_to_qr_flag = true;
            ROS_INFO("Got base_to_qr information");
        }

        // Get transform from camera_link to depth_link (from the urdf model)
        if(!cam1_to_depth1_flag)
        {
            try
            {
                // Change link name if needed
                cam1_to_depth1_transform = tfBuffer.lookupTransform("camera1_depth_optical_frame", "camera1_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            cam1_to_depth1_flag = true;
            ROS_INFO("Got cam1_to_depth1 information");
        }

        if(!cam2_to_depth2_flag)
        {
            try
            {
                // Change link name if needed
                cam2_to_depth2_transform = tfBuffer.lookupTransform("camera2_depth_optical_frame", "camera2_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            cam2_to_depth2_flag = true;
            ROS_INFO("Got cam2_to_depth2 information");
        }

        // Do calculation
        calculateTransformationMatrix();
        if(cam1_done && cam2_done) break;

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
