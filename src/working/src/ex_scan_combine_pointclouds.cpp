#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2::Ptr accumulated_cloud(new pcl::PCLPointCloud2);

// Call back function left camera
void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // Convert to PCL data type
  pcl_conversions::toPCL(*msg, *cloud);
  ROS_INFO("Received new cloud, size: %lu", cloud->data.size());

  *accumulated_cloud += *cloud;
  ROS_INFO("Accumulated cloud size after add raw data: %lu", accumulated_cloud->data.size());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (accumulated_cloud);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*accumulated_cloud);
  ROS_INFO("Accumulated cloud size after apply voxel grid filter: %lu", accumulated_cloud->data.size());

  // if (pcl_cloud->size() > 0)
  // {
  //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_left;
  //     viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen left"));
  //     viewer_test_left->setBackgroundColor(0, 0, 0);
  //     viewer_test_left->addPointCloud<pcl::PointXYZ>(pcl_cloud, "all");
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exscan_lidar_reader");

  // NodeHandle
  ros::NodeHandle nh;

  ros::Rate rate(10.0);

  ros::Subscriber pointcloud_sub = nh.subscribe("/livox/lidar", 1, callback);
  ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("accumulated_pointclouds_result", 1);

  // Publish rviz marker topic

  while (ros::ok())
  {
    if(!cloud->data.empty())
    {
      sensor_msgs::PointCloud2 msg;
      pcl_conversions::fromPCL(*accumulated_cloud, msg);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "livox_frame";
      publisher.publish(msg);
      ROS_INFO("Publishing cloud size: %lu", accumulated_cloud->data.size());
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

