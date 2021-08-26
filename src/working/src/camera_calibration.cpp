#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Past tf value containers
std::vector<Eigen::Vector3d> translationStorage;
std::vector<Eigen::Vector3d> RPYStorage;
geometry_msgs::TransformStamped transformStamped1;
geometry_msgs::TransformStamped transformStamped2;

void calculateTransformationMatrix()
{
    // Output the transform on screen
    Eigen::Vector3d translation1(transformStamped1.transform.translation.x, transformStamped1.transform.translation.y, transformStamped1.transform.translation.z);

    Eigen::Quaterniond rotation1(transformStamped1.transform.rotation.w, transformStamped1.transform.rotation.x, transformStamped1.transform.rotation.y, transformStamped1.transform.rotation.z);

//    std::cout << "Translation1: x = " << translation1(0,0) << " y = " << translation1(1,0) << " z = " << translation1(2,0) << std::endl;
//    std::cout << "Rotation1: w = " << rotation1.w() << " x = " << rotation1.x() << " y = " << rotation1.y() << " z = " << rotation1.z() << std::endl;

    // Output the transform on screen
    Eigen::Vector3d translation2(transformStamped2.transform.translation.x, transformStamped2.transform.translation.y, transformStamped2.transform.translation.z);

    Eigen::Quaterniond rotation2(transformStamped2.transform.rotation.w, transformStamped2.transform.rotation.x, transformStamped2.transform.rotation.y, transformStamped2.transform.rotation.z);

//    std::cout << "Translation2: x = " << translation2(0,0) << " y = " << translation2(1,0) << " z = " << translation2(2,0) << std::endl;
//    std::cout << "Rotation2: w = " << rotation2.w() << " x = " << rotation2.x() << " y = " << rotation2.y() << " z = " << rotation2.z() << std::endl;

    // Calculate the transform from base_link to camera_link
    tf2::Vector3 translationVector1(translation1(0,0), translation1(1,0), translation1(2,0));
    tf2::Quaternion rotationVector1(rotation1.x(), rotation1.y(), rotation1.z(), rotation1.w());
    tf2::Transform transform1;
    transform1.setOrigin(translationVector1);
    transform1.setRotation(rotationVector1);

    tf2::Vector3 translationVector2(translation2(0,0), translation2(1,0), translation2(2,0));
    tf2::Quaternion rotationVector2(rotation2.x(), rotation2.y(), rotation2.z(), rotation2.w());
    tf2::Transform transform2;
    transform2.setOrigin(translationVector2);
    transform2.setRotation(rotationVector2);

    // Output the calculated transform
    tf2::Transform finalTransform;
    finalTransform = transform1 * transform2.inverse();

    std::cout << "==> Translation: " << finalTransform.getOrigin().x() << " " << finalTransform.getOrigin().y() << " " << finalTransform.getOrigin().z() << std::endl;
    std::cout << "==> Rotataion quaternion: " << finalTransform.getRotation().w() << " " << finalTransform.getRotation().x() << " " << finalTransform.getRotation().y() << " " << finalTransform.getRotation().z() << std::endl;

    Eigen::Quaterniond eigenRotationMatrix(finalTransform.getRotation().w(),
                                           finalTransform.getRotation().x(),
                                           finalTransform.getRotation().y(),
                                           finalTransform.getRotation().z());

    Eigen::Vector3d eigenRPY = eigenRotationMatrix.toRotationMatrix().eulerAngles(0,1,2);

    std::cout << "==> Raw Pitch Yaw: " << eigenRPY(0) << " " << eigenRPY(1) << " " << eigenRPY(2) << std::endl;


    // Average over time
//    Eigen::Vector3d eigenTranslation(finalTransform.getOrigin().x(),
//                                     finalTransform.getOrigin().y(),
//                                     finalTransform.getOrigin().z());
//
//    translationStorage.push_back(eigenTranslation);
//    RPYStorage.push_back(eigenRPY);
//
//    Eigen::Vector3d averagedTranslation, totalTranslation;
//    Eigen::Vector3d averagedRPY, totalRPY;
//
//    totalTranslation.setZero();
//    totalRPY.setZero();
//
//    for(int i=0; i<translationStorage.size(); i++)
//    {
//        totalTranslation += translationStorage.at(i);
//        totalRPY += RPYStorage.at(i);
//    }
//
//    averagedTranslation = totalTranslation / translationStorage.size();
//    averagedRPY = totalRPY / RPYStorage.size();

//    std::cout << "==> Averaged Translation: " << averagedTranslation(0) << " " << averagedTranslation(1) << " " << averagedTranslation(2) << std::endl;
//    std::cout << "==> Averaged Raw Pitch Yaw: " << averagedRPY(0) << " " << averagedRPY(1) << " " << averagedRPY(2) << std::endl;

    // Broadcast the transformation
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformBroadcast;

    transformBroadcast.header.stamp = ros::Time::now();
    transformBroadcast.header.frame_id = "camera1_link";
    transformBroadcast.child_frame_id = "camera2_link";
    transformBroadcast.transform.translation.x = finalTransform.getOrigin().x();
    transformBroadcast.transform.translation.y = finalTransform.getOrigin().y();
    transformBroadcast.transform.translation.z = finalTransform.getOrigin().z();
//    tf2::Quaternion qBroadcast;
//    qBroadcast.setRPY(averagedRPY(0), averagedRPY(1), averagedRPY(2));
    transformBroadcast.transform.rotation.x = finalTransform.getRotation().x();
    transformBroadcast.transform.rotation.y = finalTransform.getRotation().y();
    transformBroadcast.transform.rotation.z = finalTransform.getRotation().z();
    transformBroadcast.transform.rotation.w = finalTransform.getRotation().w();

    br.sendTransform(transformBroadcast);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_calibration");

    // NodeHandle
    ros::NodeHandle nh;

    // Steps before run:
    // + Start all nodes required to have a full tf_tree from base_link to ar_link
    // + Start ar_track_alvar node
    // + Check the names of frames (in tf_tree and code)
    // Note: There should be 2 tf_tree (1 from camera1_link to ar_link and 1 from camera2_link to ar_tag)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    translationStorage.clear();
    RPYStorage.clear();

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        // Get transform from base to ar_link (from the urdf model)
        try
        {
            // Change link name if needed
            transformStamped1 = tfBuffer.lookupTransform("camera1_link", "camera1_linkar_marker_4", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        //  Get transform from camera to ar_tag (from ar_tag_alvar)
        try
        {
            // Change link name if needed
            transformStamped2 = tfBuffer.lookupTransform("camera2_link", "camera2_linkar_marker_4", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        calculateTransformationMatrix();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
