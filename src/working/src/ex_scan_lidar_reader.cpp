#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

const double VOXEL_GRID_SIZE = 0.05;
pcl::PCLPointCloud2 cloud_downsampled;
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

int max_intensity = 0;

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

    std::cout << "Cloud attributes:" << pcl_cloud->size() << " " << pcl_cloud->at(500) << std::endl;

    max_intensity = pcl_cloud->at(0).intensity;
    for(int i=0; i<pcl_cloud->size(); i++)
    {   
        if(pcl_cloud->at(i).intensity > max_intensity) max_intensity = pcl_cloud->at(i).intensity;
    }

    std::cout << "Max intensity:" << max_intensity << std::endl;

    // if (pcl_cloud->size() > 0)
    // {
    //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_left;
    //     viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen left"));
    //     viewer_test_left->setBackgroundColor(0, 0, 0);
    //     viewer_test_left->addPointCloud<pcl::PointXYZI>(pcl_cloud, "all");
    //     viewer_test_left->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    //     viewer_test_left->addCoordinateSystem(1.0, "World");
    //     viewer_test_left->initCameraParameters();

    //     while (!viewer_test_left->wasStopped())
    //     {
    //         viewer_test_left->spinOnce(100);
    //         ros::Duration(0.1).sleep();
    //     }
    // }

    // ROS_INFO("Received left point cloud");
}

void intensityVisualizer(pcl::PointCloud<pcl::PointXYZI> &points, ros::Publisher &publisher)
{
    visualization_msgs::Marker voxelized_object;
    voxelized_object.header.stamp = ros::Time::now();
    voxelized_object.header.frame_id = "map";
    voxelized_object.type = 6;
    voxelized_object.action = 0;
    voxelized_object.scale.x = 0.1/2;
    voxelized_object.scale.y = 0.1/2;
    voxelized_object.scale.z = 0.1/2;
    voxelized_object.pose.position.x = 0;
    voxelized_object.pose.position.y = 0;
    voxelized_object.pose.position.z = 0;
    voxelized_object.pose.orientation.x = 0;
    voxelized_object.pose.orientation.y = 0;
    voxelized_object.pose.orientation.z = 0;
    voxelized_object.pose.orientation.w = 1;

    voxelized_object.points.resize(points.size());
    voxelized_object.colors.resize(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        voxelized_object.points.at(i).x = points.at(i).x;
        voxelized_object.points.at(i).y = points.at(i).y;
        voxelized_object.points.at(i).z = points.at(i).z;
        voxelized_object.colors.at(i).r = points.at(i).intensity / 10;
        voxelized_object.colors.at(i).g = points.at(i).intensity / 10;
        voxelized_object.colors.at(i).b = points.at(i).intensity / 10;
        voxelized_object.colors.at(i).a = 1;
    }

    publisher.publish(voxelized_object);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "exscan_lidar_reader");

    // NodeHandle
    ros::NodeHandle nh;

    ros::Rate rate(10.0);

    ros::Subscriber pointcloud_sub = nh.subscribe("/livox/lidar", 1, callback);
    ros::Publisher publisher = nh.advertise<visualization_msgs::Marker>("intensity_visualisation", 1000);

    // Publish rviz marker topic

    while (ros::ok())
    {
        if(!pcl_cloud->empty())
        {
            intensityVisualizer(*pcl_cloud, publisher);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
