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
geometry_msgs::Pose cam_to_qr_pose;
// Flag values to prevent unnecessary variable update
bool status_flag = false;
bool base_to_qr_flag = false;
bool cam_to_qr_flag = false;

void generateTransformationMatrix(Eigen::Vector3d& translation, Eigen::Quaterniond& rotation, tf2::Transform& matrix)
{
    tf2::Vector3 translation_tf2(translation(0,0), translation(1,0), translation(2,0));
    tf2::Quaternion rotation_tf2(rotation.x(), rotation.y(), rotation.z(), rotation.w());
    
    matrix.setOrigin(translation_tf2);
    matrix.setRotation(rotation_tf2);
}

void qrStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{ 
    if(!status_flag && msg->data == 3)
    {
        status_flag = true;
        ROS_INFO("Visp is tracking QR");
    }
}

void qrPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // If visp status is 3 (Tracking QR)
    if(!cam_to_qr_flag && status_flag)
    {
        cam_to_qr_pose = msg->pose;

        cam_to_qr_flag = true;
        ROS_INFO("Got cam_to_qr information");

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_broadcast;

        transform_broadcast.header.stamp = ros::Time::now();
        transform_broadcast.header.frame_id = "right_camera_depth_optical_frame";
        transform_broadcast.child_frame_id = "visp_qr_frame";
        transform_broadcast.transform.translation.x = cam_to_qr_pose.position.x;
        transform_broadcast.transform.translation.y = cam_to_qr_pose.position.y;
        transform_broadcast.transform.translation.z = cam_to_qr_pose.position.z;

        transform_broadcast.transform.rotation.x = cam_to_qr_pose.orientation.x;
        transform_broadcast.transform.rotation.y = cam_to_qr_pose.orientation.y;
        transform_broadcast.transform.rotation.z = cam_to_qr_pose.orientation.z;
        transform_broadcast.transform.rotation.w = cam_to_qr_pose.orientation.w;

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
    status_flag = false;
    base_to_qr_flag = false;
    cam_to_qr_flag = false;
}

void calculateTransformationMatrix()
{
    // Process base_to_qr transform from tf_tree

    if (base_to_qr_flag && cam_to_qr_flag)
    {
        tf2::Transform base_to_qr_matrix;

        Eigen::Vector3d base_to_qr_trans(base_to_qr_transform.transform.translation.x, base_to_qr_transform.transform.translation.y, base_to_qr_transform.transform.translation.z);
        Eigen::Quaterniond base_to_qr_rot(base_to_qr_transform.transform.rotation.w, base_to_qr_transform.transform.rotation.x, base_to_qr_transform.transform.rotation.y, base_to_qr_transform.transform.rotation.z);

        std::cout << base_to_qr_trans << std::endl;
        std::cout << base_to_qr_rot.w() << " " << base_to_qr_rot.x() << " " << base_to_qr_rot.y() << "" << base_to_qr_rot.z() << std::endl;

        generateTransformationMatrix(base_to_qr_trans, base_to_qr_rot, base_to_qr_matrix);

        // Process cam_to_qr transform from visp_auto_tracker/object_position topic
        tf2::Transform cam_to_qr_matrix;

        Eigen::Vector3d cam_to_qr_trans(cam_to_qr_pose.position.x, cam_to_qr_pose.position.y, cam_to_qr_pose.position.z);
        Eigen::Quaterniond cam_to_qr_rot(cam_to_qr_pose.orientation.w, cam_to_qr_pose.orientation.x, cam_to_qr_pose.orientation.y, cam_to_qr_pose.orientation.z);

        generateTransformationMatrix(cam_to_qr_trans, cam_to_qr_rot, cam_to_qr_matrix);

        std::cout << cam_to_qr_trans << std::endl;
        std::cout << cam_to_qr_rot.w() << " " << cam_to_qr_rot.x() << " " << cam_to_qr_rot.y() << " " << cam_to_qr_rot.z() << std::endl;


        // Calculate base_to_cam transform
        tf2::Transform base_to_cam_matrix;
        base_to_cam_matrix = base_to_qr_matrix * cam_to_qr_matrix.inverse();

        ROS_INFO("Camera to base_link transform: ");
        std::cout << "==> Translation: " << base_to_cam_matrix.getOrigin().x() << " " << base_to_cam_matrix.getOrigin().y() << " " << base_to_cam_matrix.getOrigin().z() << std::endl;
        std::cout << "==> Rotataion quaternion: " << base_to_cam_matrix.getRotation().w() << " " << base_to_cam_matrix.getRotation().x() << " " << base_to_cam_matrix.getRotation().y() << " " << base_to_cam_matrix.getRotation().z() << std::endl;

        broadcastTransform(base_to_cam_matrix, "base_link", "mock_camera");
        resetFlags();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_calibration_qr");

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
    ros::Subscriber sub_qr_pos = nh.subscribe("/visp_auto_tracker/object_position", 1, qrPosCallback);
    ros::Subscriber sub_qr_status = nh.subscribe("/visp_auto_tracker/status", 1, qrStatusCallback);

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
                base_to_qr_transform = tfBuffer.lookupTransform("base_link", "qr_link", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            base_to_qr_flag = true;
            ROS_INFO("Got base_to_qr information");
        }


        calculateTransformationMatrix();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
