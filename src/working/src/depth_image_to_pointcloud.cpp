#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"
#include "vector"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "opencv2/highgui/highgui.hpp"
#include "pcl/filters/passthrough.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/ply_io.h>

#include <stdint.h>

// Global variables
double cx_depth, cy_depth, cx_color, cy_color; //Principal point
double fx_depth, fy_depth, fx_color, fy_color; // Focal length
cv_bridge::CvImagePtr cv_ptr_depth (new cv_bridge::CvImage); // Depth image pointer, cv::Mat form
cv_bridge::CvImagePtr cv_ptr_color (new cv_bridge::CvImage); // Color image pointer, cv::Mat form
pcl::PointCloud<pcl::PointXYZ>::Ptr gen_cloud (new pcl::PointCloud<pcl::PointXYZ>); // Generated point cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr gen_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>); // Generated point cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // Generated combined point cloud for testing purpose;

void imageCallbackColor(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr_color = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void imageInfoCallbackColor(const sensor_msgs::CameraInfoPtr& msg)
{
    cx_color = msg->P.at(2);
    cy_color = msg->P.at(6);
    fx_color = msg->P.at(0);
    fy_color = msg->P.at(5);
}

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

//    cv::Mat imageUpper(480,640,CV_8UC1);
//    cv::Mat imageLower(480,640,CV_8UC1);
//    for (int i = 0; i < cv_ptr_depth->image.rows; i++)
//    {
//        for (int j = 0; j < cv_ptr_depth->image.cols; j++)
//        {
//            imageUpper.at<uchar>(i,j) = static_cast<uchar>(cv_ptr_depth->image.at<unsigned short>(i,j)/256);
//            imageLower.at<uchar>(i,j) = static_cast<uchar>(cv_ptr_depth->image.at<unsigned short>(i,j) - (((unsigned short)(cv_ptr_depth->image.at<unsigned short>(i,j)/256))*256));
//        }
//    }
//
//    std::cout << "Image type: " << cv_ptr_depth->image.type() << std::endl;
//    cv::imshow("image", imageUpper);
//    cv::imshow("image_2", imageLower);
//    cv::waitKey(10);
}

void imageInfoCallbackDepth(const sensor_msgs::CameraInfoPtr& msg)
{
    cx_depth = msg->P.at(2);
    cy_depth = msg->P.at(6);
    fx_depth = msg->P.at(0);
    fy_depth = msg->P.at(5);
}

// --------------
// -----Generate colored point cloud from depth and color image-----
// --------------
void addColorToPointCloud()
{
    gen_cloud_color->clear();
    int row_color, col_color;
    cv::Point3_<uchar>* image_pixel;

    pcl::PointXYZRGB point_color;
    for (int i=0; i<gen_cloud->size(); i++)
    {
        // Use current point xyz to find corresponding pixel in colored image
        point_color.x = gen_cloud->at(i).x;
        point_color.y = gen_cloud->at(i).y;
        point_color.z = gen_cloud->at(i).z;

        col_color = ((gen_cloud->at(i).x / gen_cloud->at(i).z * fx_color) + cx_color);
        row_color = ((gen_cloud->at(i).y / gen_cloud->at(i).z * fy_color) + cy_color);

        // If the pixel values are reasonable, add colored point to final cloud
        if((row_color >= 0) && (row_color < 480) && (col_color >= 0)  && (col_color < 640))
        {
            image_pixel = cv_ptr_color->image.ptr< cv::Point3_<uchar> >(row_color,col_color);
            uint8_t r = image_pixel->x;
            uint8_t g = image_pixel->y;
            uint8_t b = image_pixel->z;
            uint32_t rgb = (uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b;
            point_color.rgb = *reinterpret_cast<float*>(&rgb);
            gen_cloud_color->push_back(point_color);
        }
    }
}

// --------------
// -----Generate colored point cloud from depth and color image-----
// --------------
void generatePointCloud()
{
    // Generate non-color point cloud from depth-image
    gen_cloud->clear();

    pcl::PointXYZ point_3D;

    // Calculate xyz values for each pixel in the depth image
    for(int row=0; row<cv_ptr_depth->image.rows; row++)
    {
        for(int col=0; col<cv_ptr_depth->image.cols; col++)
        {
            if ((static_cast<float>(cv_ptr_depth->image.at<unsigned short>(row,col))!=0))
            {
                point_3D.x = (col - cx_depth) / fx_depth * (static_cast<float>(cv_ptr_depth->image.at<unsigned short>(row,col)) * 0.001);
                point_3D.y = (row - cy_depth) / fy_depth * (static_cast<float>(cv_ptr_depth->image.at<unsigned short>(row,col)) * 0.001);
                point_3D.z = static_cast<float>(cv_ptr_depth->image.at<unsigned short>(row,col)) * 0.001;
                gen_cloud->push_back(point_3D);
            }
        }
    }

//    std::cout << "Sample pixel depth value: " << (cv_ptr_depth->image.at<unsigned short>(100,100)) << " " << (cv_ptr_depth->image.at<unsigned short>(200,200)) << std::endl;

    /**************************Filter by range*************************/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(gen_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    pass.filter(*gen_cloud);

    // Adds the color information to the point cloud
    addColorToPointCloud();

//    // Visualize the result
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test;
    viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    viewer_test->setBackgroundColor(0, 0, 0);
    viewer_test->addPointCloud(gen_cloud_color, "all");
    viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
//    viewer_test->addCoordinateSystem(1.0, "World");
    viewer_test->initCameraParameters();

    while (!viewer_test->wasStopped()) {
        viewer_test->spinOnce(100);
        ros::Duration(0.1).sleep();
    }
}

// --------------
// -----Global variables for camera generated point cloud-----
// --------------
pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
pcl::PCLPointCloud2 cloud_downsampled;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

float voxel_grid_size = 0.01f;

// --------------
// -----Point Cloud Callback function-----
// --------------
void convertRosMSGPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Downsampling input point cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    sor.filter(cloud_downsampled);

    pcl::fromPCLPointCloud2(cloud_downsampled, *cloud_filtered);

    /**************************Filter by range*************************/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 5.0);
    pass.filter(*cloud_filtered);

//    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test;
//    viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
//    viewer_test->setBackgroundColor(0, 0, 0);
//    viewer_test->addPointCloud(cloud_filtered, "all");
//    viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
////    viewer_test->addCoordinateSystem(1.0, "World");
//    viewer_test->initCameraParameters();
//
//    while (!viewer_test->wasStopped()) {
//        viewer_test->spinOnce(100);
//        ros::Duration(0.1).sleep();
//    }
}

// --------------
// -----Point cloud combination function for testing purpose-----
// --------------
void combinedCloudGen()
{
    combined_cloud->clear();

    pcl::PointXYZRGB temp_point;
    uint32_t camera_cloud_color = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
    uint32_t depth_cloud_color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);

    for(int i=0; i<gen_cloud->size(); i++)
    {
        temp_point.rgb = *reinterpret_cast<float*>(&depth_cloud_color);
        temp_point.x = gen_cloud->at(i).x;
        temp_point.y = gen_cloud->at(i).y;
        temp_point.z = gen_cloud->at(i).z;
        combined_cloud->push_back(temp_point);
    }

    for(int i=0; i<cloud_filtered->size(); i++)
    {
        temp_point.rgb = *reinterpret_cast<float*>(&camera_cloud_color);
        temp_point.x = cloud_filtered->at(i).x;
        temp_point.y = cloud_filtered->at(i).y;
        temp_point.z = cloud_filtered->at(i).z;
        combined_cloud->push_back(temp_point);
    }

//    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test;
//    viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
//    viewer_test->setBackgroundColor(0, 0, 0);
//    viewer_test->addPointCloud(combined_cloud, "all");
//    viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
////    viewer_test->addCoordinateSystem(1.0, "World");
//    viewer_test->initCameraParameters();
//
//    while (!viewer_test->wasStopped()) {
//        viewer_test->spinOnce(100);
//        ros::Duration(0.1).sleep();
//    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber subDepthImage = it.subscribe("/camera/depth/image_rect_raw", 1, imageCallbackDepth);
    ros::Subscriber subDepthImageInfo = n.subscribe("/camera/depth/camera_info", 1, imageInfoCallbackDepth);

    image_transport::Subscriber subColorImage = it.subscribe("/camera/color/image_raw", 1, imageCallbackColor);
    ros::Subscriber subColorImageInfo = n.subscribe("/camera/color/camera_info", 1, imageInfoCallbackColor);

    ros::Subscriber pointcloud_sub = n.subscribe("/camera/depth/color/points", 1, convertRosMSGPointCloud);


    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (cv_ptr_depth->image.cols > 0 && cv_ptr_color->image.cols > 0) {
            generatePointCloud();
//            combinedCloudGen();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
