#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

const double VOXEL_GRID_SIZE = 0.05;
pcl::PCLPointCloud2 cloud_downsampled;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Call back function left camera
void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // // Downsampling input point cloud
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(cloudPtr);
    // sor.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    // sor.filter(cloud_downsampled);

    pcl::fromPCLPointCloud2(*cloud, *pcl_cloud);

    std::cout << "Cloud attributes:" << pcl_cloud->size() << " " << pcl_cloud->empty() << std::endl;

    if (pcl_cloud->size() > 0)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_left;
        viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen left"));
        viewer_test_left->setBackgroundColor(0, 0, 0);
        viewer_test_left->addPointCloud(pcl_cloud, "all");
        viewer_test_left->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
        viewer_test_left->addCoordinateSystem(1.0, "World");
        viewer_test_left->initCameraParameters();

        while (!viewer_test_left->wasStopped())
        {
            viewer_test_left->spinOnce(100);
            ros::Duration(0.1).sleep();
        }
    }

    // ROS_INFO("Received left point cloud");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exscan_lidar_reader");

    // NodeHandle
    ros::NodeHandle nh;

    ros::Rate rate(10.0);

    // Get point cloud data of each camera - Using 1 camera for now
    ros::Subscriber left_pointcloud_sub = nh.subscribe("/livox/lidar", 1, callback);

    // Publish rviz marker topic

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
