#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/extract_indices.h"

#include "vox-file/vox_file.h"

#include <iostream>
#include <string>
#include <random>
#include <Eigen/Dense>

/**
 * This code convert stl file into marker msg which can be displayed in rviz
 */

//Structs
struct Color
{
    /* data */
    float r,g,b,a;
};


struct VoxelizedObj
{
    /* data */
    pcl::PointCloud<pcl::PointXYZ> positions;
    std::vector<Color> rgbas;
    pcl::PointCloud<pcl::PointNormal> normals;
};

// Constants
const std::string OBJECT_FILE = "/home/long/phd/src/working/models/Truck/voxelized_truck.ply";
const std::string VOXEL_FILE = "/home/long/phd/src/working/models/Truck/truck.vox";
const float VOXEL_SIZE = 1;
const double BUCKET_BOUND_X_MAX = 145;
const double BUCKET_BOUND_X_MIN = 20;
const double BUCKET_BOUND_X_SD = 5;
const double BUCKET_BOUND_Y_MAX = 196;
const double BUCKET_BOUND_Y_MIN = 60;
const double BUCKET_BOUND_Y_SD = 5;
const double BUCKET_BOUND_Z_MAX = 114;
const double BUCKET_BOUND_Z_MIN = 70;
const double BUCKET_BOUND_Z_SD = 5;
const double NORMAL_SCALE = 2;

int mode;

// Get the normal distribution value
void calDistValue(double mean1, double mean2, double sd, double input, double& output)
{
    double output1 = 1 / (sd * sqrt(2*M_PI)) * exp(-pow((input - mean1),2.0)/(2 * pow(sd, 2.0)));
    double output2 = 1 / (sd * sqrt(2*M_PI)) * exp(-pow((input - mean2),2.0)/(2 * pow(sd, 2.0)));
    output = output1 + output2;
}

// Choose color for the voxel
void chooseColorVox(VoxelizedObj &obj)
{
    obj.rgbas.resize(obj.positions.size());
    for(int i=0; i<obj.rgbas.size(); i++)
    {
        obj.rgbas.at(i).r = 0;
        obj.rgbas.at(i).b = 0;
        obj.rgbas.at(i).g = 0;
        obj.rgbas.at(i).a = 1;
    }

    for (int i = 0; i < obj.positions.size(); i++)
    {
        if (mode == 2)
        {
            // r based on x
            double metric_x, metric_x_max;
            calDistValue(BUCKET_BOUND_X_MAX, BUCKET_BOUND_X_MIN, BUCKET_BOUND_X_SD, obj.positions.at(i).x, metric_x);
            calDistValue(BUCKET_BOUND_X_MAX, BUCKET_BOUND_X_MIN, BUCKET_BOUND_X_SD, BUCKET_BOUND_X_MAX, metric_x_max);
            obj.rgbas.at(i).r = 255 * metric_x / metric_x_max;

            // g based on y
            double metric_y, metric_y_max;
            calDistValue(BUCKET_BOUND_Y_MAX, BUCKET_BOUND_Y_MIN, BUCKET_BOUND_Y_SD, obj.positions.at(i).y, metric_y);
            calDistValue(BUCKET_BOUND_Y_MAX, BUCKET_BOUND_Y_MIN, BUCKET_BOUND_Y_SD, BUCKET_BOUND_Y_MAX, metric_y_max);
            obj.rgbas.at(i).g = 255 * metric_y / metric_y_max;

            // b based on z
            double metric_z, metric_z_max;
            calDistValue(BUCKET_BOUND_Z_MAX, BUCKET_BOUND_Z_MIN, BUCKET_BOUND_Z_SD, obj.positions.at(i).z, metric_z);
            calDistValue(BUCKET_BOUND_Z_MAX, BUCKET_BOUND_Z_MIN, BUCKET_BOUND_Z_SD, BUCKET_BOUND_Z_MAX, metric_z_max);
            obj.rgbas.at(i).b = 255 * metric_z / metric_z_max;

            // obj.rgbas.at(i).r = 255 * (metric_x/metric_x_max * metric_y/metric_y_max * metric_z/metric_z_max);
            // obj.rgbas.at(i).g = 255 * (metric_x/metric_x_max * metric_y/metric_y_max * metric_z/metric_z_max);
            // obj.rgbas.at(i).b = 255 * (metric_x/metric_x_max * metric_y/metric_y_max * metric_z/metric_z_max);
        }
        else if (mode == 1)
        {
            if (obj.positions.at(i).x == BUCKET_BOUND_X_MAX || obj.positions.at(i).x == BUCKET_BOUND_X_MIN)
                obj.rgbas.at(i).r = 255;
            if (obj.positions.at(i).y == BUCKET_BOUND_Y_MAX || obj.positions.at(i).y == BUCKET_BOUND_Y_MIN)
                obj.rgbas.at(i).g = 255;
            if (obj.positions.at(i).z == BUCKET_BOUND_Z_MAX || obj.positions.at(i).z == BUCKET_BOUND_Z_MIN)
                obj.rgbas.at(i).b = 255;
        }
    }
}

// Load magicavoxel file
void loadVoxelFile(VoxelizedObj &obj)
{
    magicavoxel::VoxFile vox_file(true, true);
    vox_file.Load(VOXEL_FILE);
    cout << "There are " << vox_file.sparseModels().at(0).voxels().size() << " voxels in the first model." << endl;
    magicavoxel::VoxSparseModel sparse_model = vox_file.sparseModels().at(0);

    obj.positions.resize(sparse_model.voxels().size());

    for(int i=0; i<sparse_model.voxels().size(); i++)
    {
        obj.positions.at(i).x = (unsigned short)sparse_model.voxels().at(i).x;
        obj.positions.at(i).y = (unsigned short)sparse_model.voxels().at(i).y;
        obj.positions.at(i).z = (unsigned short)sparse_model.voxels().at(i).z;
    }

    chooseColorVox(obj);

    unsigned short minx, miny, minz, maxx, maxy, maxz;
    minx = 128;
    miny = minx;
    minz = minx;
    maxx = 128;
    maxy = maxx;
    maxz = maxx;
    for(int i=0; i<sparse_model.voxels().size(); i++)
    {
        if((unsigned short)sparse_model.voxels().at(i).x < minx) minx = sparse_model.voxels().at(i).x;
        if((unsigned short)sparse_model.voxels().at(i).y < miny) miny = sparse_model.voxels().at(i).y;
        if((unsigned short)sparse_model.voxels().at(i).z < minz) minz = sparse_model.voxels().at(i).z;
        if((unsigned short)sparse_model.voxels().at(i).x > maxx) maxx = sparse_model.voxels().at(i).x;
        if((unsigned short)sparse_model.voxels().at(i).y > maxy) maxy = sparse_model.voxels().at(i).y;
        if((unsigned short)sparse_model.voxels().at(i).z > maxz) maxz = sparse_model.voxels().at(i).z;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // *cloud_ptr = obj.positions;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_left;
    // viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    // viewer_test_left->setBackgroundColor(0, 0, 0);
    // viewer_test_left->addPointCloud(cloud_ptr, "all");
    // viewer_test_left->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    // viewer_test_left->initCameraParameters();

    // while (!viewer_test_left->wasStopped())
    // {
    //     viewer_test_left->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

// Load stl file into point cloud
void loadObjectFile(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::io::loadPLYFile(OBJECT_FILE, cloud);
    std::cout << "Number of points from ply file: " << cloud.size() << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // *cloud_ptr = cloud;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_left;
    // viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    // viewer_test_left->setBackgroundColor(0, 0, 0);
    // viewer_test_left->addPointCloud(cloud_ptr, "all");
    // viewer_test_left->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    // viewer_test_left->initCameraParameters();

    // while (!viewer_test_left->wasStopped())
    // {
    //     viewer_test_left->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

// Voxelize the point cloud
void voxelizeObject(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_ptr = cloud;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    sor.filter(cloud);

    // *cloud_ptr = cloud;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_left;
    // viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    // viewer_test_left->setBackgroundColor(0, 0, 0);
    // viewer_test_left->addPointCloud(cloud_ptr, "all");
    // viewer_test_left->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    // viewer_test_left->initCameraParameters();

    // while (!viewer_test_left->wasStopped())
    // {
    //     viewer_test_left->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

// Visualize the result with markers in rviz
void visualizeResult(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Publisher& publisher)
{
    visualization_msgs::Marker voxelized_object;
    voxelized_object.header.stamp = ros::Time::now();
    voxelized_object.header.frame_id = "random_frame";
    voxelized_object.type = 6;
    voxelized_object.action = 0;
    voxelized_object.scale.x = VOXEL_SIZE;
    voxelized_object.scale.y = VOXEL_SIZE;
    voxelized_object.scale.z = VOXEL_SIZE;
    voxelized_object.pose.position.x = 0;
    voxelized_object.pose.position.y = 0;
    voxelized_object.pose.position.z = 0;
    voxelized_object.pose.orientation.x = 0;
    voxelized_object.pose.orientation.y = 0;
    voxelized_object.pose.orientation.z = 0;
    voxelized_object.pose.orientation.w = 1;

    voxelized_object.points.resize(cloud.size());
    voxelized_object.colors.resize(cloud.size());
    for(int i=0; i<cloud.size(); i++)
    {
        voxelized_object.points.at(i).x = cloud.at(i).x;
        voxelized_object.points.at(i).y = cloud.at(i).y;
        voxelized_object.points.at(i).z = cloud.at(i).z;
        voxelized_object.colors.at(i).r = 0;
        voxelized_object.colors.at(i).g = 255;
        voxelized_object.colors.at(i).b = 0;
        voxelized_object.colors.at(i).a = 1;
    }

    publisher.publish(voxelized_object);
}

void calculateNormals(VoxelizedObj &obj)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = obj.positions;
    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (cloud->isOrganized())
    {
        tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    }
    else
    {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud);

    // Compute normals 
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

    // calculate normals with the small scale
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(NORMAL_SCALE);
    ne.compute(*normals);

    obj.normals = *normals;

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test;
    // viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    // viewer_test->setBackgroundColor(0, 0, 0);
    // viewer_test->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, normals, 100, 5, "all", 0);
    // viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(100.0, "World");
    // viewer_test->initCameraParameters();

    // while (!viewer_test->wasStopped())
    // {
    //     viewer_test->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

void keepEdges(VoxelizedObj &obj)
{
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    indices->indices.clear();

    Eigen::Vector3f unit_vec(0, 0, 1);

    for(int i=0; i<obj.normals.size(); i++)
    {
        Eigen::Vector3f normal_vec(obj.normals.at(i).normal_x, obj.normals.at(i).normal_y, obj.normals.at(i).normal_z);  
        float angle = normal_vec.dot(unit_vec) / normal_vec.norm();      
        if((angle >= cos(DEG2RAD(130)) && angle <= cos(DEG2RAD(50))) || angle <= cos(DEG2RAD(140)) || angle >= cos(DEG2RAD(40)))
            indices->indices.push_back(i);
        // std::cout << "Angle values: " << cos(DEG2RAD(100)) << cos(DEG2RAD(80)) << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    *cloud = obj.positions;

    pcl::ExtractIndices<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setIndices(indices);
    // The resulting cloud_out is identical to cloud_in, but all points referenced by indices_in are made NaN:
    filter.setNegative(true);
    filter.setKeepOrganized(false);
    filter.filter(*cloud_filtered);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test;
    viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    viewer_test->setBackgroundColor(0, 0, 0);
    viewer_test->addPointCloud(cloud_filtered, "all");
    viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    viewer_test->addCoordinateSystem(100.0, "World");
    viewer_test->initCameraParameters();

    while (!viewer_test->wasStopped())
    {
        viewer_test->spinOnce(100);
        ros::Duration(0.1).sleep();
    }
}

void visualizeVoxel(VoxelizedObj &obj, ros::Publisher& publisher)
{
    visualization_msgs::Marker voxelized_object;
    voxelized_object.header.stamp = ros::Time::now();
    voxelized_object.header.frame_id = "random_frame";
    voxelized_object.type = 6;
    voxelized_object.action = 0;
    // voxelized_object.scale.x = VOXEL_SIZE/2;
    // voxelized_object.scale.y = VOXEL_SIZE/2;
    // voxelized_object.scale.z = VOXEL_SIZE/2;
    voxelized_object.scale.x = 1;
    voxelized_object.scale.y = 1;
    voxelized_object.scale.z = 1;
    voxelized_object.pose.position.x = 0;
    voxelized_object.pose.position.y = 0;
    voxelized_object.pose.position.z = 0;
    voxelized_object.pose.orientation.x = 0;
    voxelized_object.pose.orientation.y = 0;
    voxelized_object.pose.orientation.z = 0;
    voxelized_object.pose.orientation.w = 1;

    voxelized_object.points.resize(obj.positions.size());
    voxelized_object.colors.resize(obj.positions.size());
    for(int i=0; i<obj.positions.size(); i++)
    {
        voxelized_object.points.at(i).x = obj.positions.at(i).x - 128;
        voxelized_object.points.at(i).y = obj.positions.at(i).y - 128;
        voxelized_object.points.at(i).z = obj.positions.at(i).z;
        voxelized_object.colors.at(i).r = obj.rgbas.at(i).r/256;
        voxelized_object.colors.at(i).g = obj.rgbas.at(i).g/256;
        voxelized_object.colors.at(i).b = obj.rgbas.at(i).b/256;
        voxelized_object.colors.at(i).a = obj.rgbas.at(i).a;
    }

    publisher.publish(voxelized_object);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_voxelizer");

    ros::NodeHandle n;

    ros::Publisher publisher = n.advertise<visualization_msgs::Marker>("voxelized_object", 1000);

    ros::Rate loop_rate(10);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    VoxelizedObj obj;

    mode = 1;
    loadVoxelFile(obj);

    while (ros::ok())
    {
        // loadObjectFile(cloud);
        // voxelizeObject(cloud);
        // visualizeResult(cloud, publisher);

        ROS_INFO("Enter display mode:\n 1. Limits\n 2. Metric distribution");
        std::cin >> mode;
        loadVoxelFile(obj);
        calculateNormals(obj);
        keepEdges(obj);
        visualizeVoxel(obj,publisher);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}