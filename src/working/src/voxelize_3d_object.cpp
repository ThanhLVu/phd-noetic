#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "pcl/io/ply_io.h"
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/features/don.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/kdtree/flann.h"

#include "vox-file/vox_file.h"
#include "fov_check.h"
#include "generate_viewpoint.h"

#include <iostream>
#include <string>
#include <random>
#include <Eigen/Dense>

/**
 * This code convert stl file into marker msg which can be displayed in rviz
 */

// Structs
struct Color
{
    /* data */
    float r, g, b, a;
};

struct VoxelizedObj
{
    /* data */
    pcl::PointCloud<pcl::PointXYZ> positions;
    std::vector<Color> rgbas;
    pcl::PointCloud<pcl::PointNormal> normals;
    std::vector<double> weighting;
};

// Constants
const std::string OBJECT_FILE = "/home/long/phd/src/working/models/Truck/voxelized_truck.ply";
const std::string VOXEL_FILE = "/home/long/phd/src/working/models/Truck/truck.vox";
const float VOXEL_SIZE = 0.5;
const double BUCKET_BOUND_X_MAX = 145;
const double BUCKET_BOUND_X_MIN = 12;
const double BUCKET_BOUND_X_SD = 5;
const double POS_X_RANGE = 10;
const double BUCKET_BOUND_Y_MAX = 196;
const double BUCKET_BOUND_Y_MIN = 60;
const double BUCKET_BOUND_Y_SD = 5;
const double POS_Y_RANGE = 10;
const double BUCKET_BOUND_Z_MAX = 114;
const double BUCKET_BOUND_Z_MIN = 70;
const double BUCKET_BOUND_Z_SD = 5;
const double POS_Z_RANGE = 10;
const double NORMAL_SEARCH_RADIUS = 4;
const double IMMEDIATE_NEIGHBOUR_RADIUS = 4;

int mode;

// Calculate angle difference between 2 normals
void calNormalDif(pcl::PointNormal point1, pcl::PointNormal point2, double &result)
{
    Eigen::Vector3d vec1(point1.normal_x, point1.normal_y, point1.normal_z);
    Eigen::Vector3d vec2(point2.normal_x, point2.normal_y, point2.normal_z);

    result = fabs(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

// Get the normal distribution value
void calPosWeightv1(double mean1, double mean2, double sd, double input, double &output)
{
    double output1 = 1 / (sd * sqrt(2 * M_PI)) * exp(-pow((input - mean1), 2.0) / (2 * pow(sd, 2.0)));
    double output2 = 1 / (sd * sqrt(2 * M_PI)) * exp(-pow((input - mean2), 2.0) / (2 * pow(sd, 2.0)));
    output = output1 + output2;
}

// Generate position weight
// Note: peak1 < peak2
void calPosWeightv2(double input, double peak1, double peak2, double range, double &output)
{
    if(peak1 > peak2)
    {
        ROS_ERROR("Input error, condition: peak1 < peak2");
        return;
    }

    double temp_out1, temp_out2;

    double lower_range1 = peak1 - range;
    double upper_range1 = peak1 + range;
    if(input < lower_range1) temp_out1 = 0;
    if(lower_range1 <= input)
        if (input <= upper_range1) temp_out1 = 1 - fabs(peak1 - input)/range; 
    if(upper_range1 < input) temp_out1 = 0;

    double lower_range2 = peak2 - range;
    double upper_range2 = peak2 + range;
    if(input < lower_range2) temp_out2 = 0;
    if(lower_range2 <= input)
        if (input <= upper_range2) temp_out2 = 1 - fabs(peak2 - input)/range; 
    if(upper_range2 < input) temp_out2 = 0;
    
    output = temp_out1 + temp_out2;
}

// Choose color for the voxel
void chooseColorVox(VoxelizedObj &obj)
{
    obj.rgbas.resize(obj.positions.size());
    for (int i = 0; i < obj.rgbas.size(); i++)
    {
        obj.rgbas.at(i).r = 0;
        obj.rgbas.at(i).b = 0;
        obj.rgbas.at(i).g = 0;
        obj.rgbas.at(i).a = 1;
    }

    double max;
    max = 0;
    int max_idx;

    for (int i = 0; i < obj.positions.size(); i++)
    {
        if(obj.positions.at(i).x < BUCKET_BOUND_X_MIN - POS_X_RANGE || obj.positions.at(i).x > BUCKET_BOUND_X_MAX + POS_X_RANGE)
            continue;
        if(obj.positions.at(i).y < BUCKET_BOUND_Y_MIN - POS_Y_RANGE || obj.positions.at(i).y > BUCKET_BOUND_Y_MAX + POS_Y_RANGE)
            continue;
        if(obj.positions.at(i).z < BUCKET_BOUND_Z_MIN - POS_Z_RANGE || obj.positions.at(i).z > BUCKET_BOUND_Z_MAX + POS_Z_RANGE)
            continue;
        if (mode == 1)
        {
            // r based on x
            double metric_x, metric_x_max;
            calPosWeightv1(BUCKET_BOUND_X_MAX, BUCKET_BOUND_X_MIN, BUCKET_BOUND_X_SD, obj.positions.at(i).x, metric_x);
            calPosWeightv1(BUCKET_BOUND_X_MAX, BUCKET_BOUND_X_MIN, BUCKET_BOUND_X_SD, BUCKET_BOUND_X_MAX, metric_x_max);
            obj.rgbas.at(i).r = 255 * metric_x / metric_x_max;

            // g based on y
            double metric_y, metric_y_max;
            calPosWeightv1(BUCKET_BOUND_Y_MAX, BUCKET_BOUND_Y_MIN, BUCKET_BOUND_Y_SD, obj.positions.at(i).y, metric_y);
            calPosWeightv1(BUCKET_BOUND_Y_MAX, BUCKET_BOUND_Y_MIN, BUCKET_BOUND_Y_SD, BUCKET_BOUND_Y_MAX, metric_y_max);
            obj.rgbas.at(i).g = 255 * metric_y / metric_y_max;

            // b based on z
            double metric_z, metric_z_max;
            calPosWeightv1(BUCKET_BOUND_Z_MAX, BUCKET_BOUND_Z_MIN, BUCKET_BOUND_Z_SD, obj.positions.at(i).z, metric_z);
            calPosWeightv1(BUCKET_BOUND_Z_MAX, BUCKET_BOUND_Z_MIN, BUCKET_BOUND_Z_SD, BUCKET_BOUND_Z_MAX, metric_z_max);
            obj.rgbas.at(i).b = 255 * metric_z / metric_z_max;
        }
        else if (mode == 2)
        {
            double weight_x;
            calPosWeightv2(obj.positions.at(i).x, BUCKET_BOUND_X_MIN, BUCKET_BOUND_X_MAX, POS_X_RANGE, weight_x);
            obj.rgbas.at(i).r = 255 * weight_x;

            double weight_y;
            calPosWeightv2(obj.positions.at(i).y, BUCKET_BOUND_Y_MIN, BUCKET_BOUND_Y_MAX, POS_Y_RANGE, weight_y);
            obj.rgbas.at(i).g = 255 * weight_y;

            double weight_z;
            calPosWeightv2(obj.positions.at(i).z, BUCKET_BOUND_Z_MIN, BUCKET_BOUND_Z_MAX, POS_Z_RANGE, weight_z);
            obj.rgbas.at(i).b = 255 * weight_z;
        }        
        else if (mode == 3)
        {
            if (obj.normals.at(i).curvature > 0.7)
            {
            obj.rgbas.at(i).r = 255;
            obj.rgbas.at(i).g = 255;
            obj.rgbas.at(i).b = 255;
            }
        }
        else if (mode == 4)
        {
            // The weighting is dot product so 1 = no normal differences
            if (obj.weighting.at(i) <= 0.92 && obj.weighting.at(i) >= 0.9)
            {
                // if (1) {
                obj.rgbas.at(i).r = 255 * obj.weighting.at(i);
                obj.rgbas.at(i).g = 255 * obj.weighting.at(i);
                obj.rgbas.at(i).b = 255 * obj.weighting.at(i);
                if (max < obj.weighting.at(i))
                {
                    max = obj.weighting.at(i);
                    max_idx = i;
                    std::cout << max_idx << " - " << max << std::endl;
                }
            }
        }
        else if (mode == 5)
        {
            double weight_x;
            calPosWeightv2(obj.positions.at(i).x, BUCKET_BOUND_X_MIN, BUCKET_BOUND_X_MAX, POS_X_RANGE, weight_x);
            
            double weight_y;
            calPosWeightv2(obj.positions.at(i).y, BUCKET_BOUND_Y_MIN, BUCKET_BOUND_Y_MAX, POS_Y_RANGE, weight_y);
            
            double weight_z;
            calPosWeightv2(obj.positions.at(i).z, BUCKET_BOUND_Z_MIN, BUCKET_BOUND_Z_MAX, POS_Z_RANGE, weight_z);

            double total_weight = (weight_x + weight_y + weight_z)/3; 
            // The weighting is dot product so 1 = no normal differences 

            if (obj.weighting.at(i) < 0.99) 
            {
                obj.rgbas.at(i).b = 255 * total_weight * obj.weighting.at(i);
                obj.rgbas.at(i).g = 255 * total_weight * obj.weighting.at(i);
                obj.rgbas.at(i).r = 255 * total_weight * obj.weighting.at(i);
            }
            if (max < total_weight)
            {
                max = total_weight;
                max_idx = i;
            }
        }
    }
    std::cout << max_idx << " - " << max << std::endl;
}

// Load magicavoxel file
void loadVoxelFile(VoxelizedObj &obj)
{
    magicavoxel::VoxFile vox_file(true, true);
    vox_file.Load(VOXEL_FILE);
    cout << "There are " << vox_file.sparseModels().at(0).voxels().size() << " voxels in the first model." << endl;
    magicavoxel::VoxSparseModel sparse_model = vox_file.sparseModels().at(0);

    obj.positions.resize(sparse_model.voxels().size());

    for (int i = 0; i < sparse_model.voxels().size(); i++)
    {
        obj.positions.at(i).x = (unsigned short)sparse_model.voxels().at(i).x;
        obj.positions.at(i).y = (unsigned short)sparse_model.voxels().at(i).y;
        obj.positions.at(i).z = (unsigned short)sparse_model.voxels().at(i).z;
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

void calDoNWeight(VoxelizedObj &obj)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
    ne.setViewPoint(0.0, 0.0, std::numeric_limits<float>::max());

    // calculate normals with the small scale
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(NORMAL_SEARCH_RADIUS / 2);
    ne.compute(*normals_small);

    // calculate normals with the large scale
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(NORMAL_SEARCH_RADIUS * 2);
    ne.compute(*normals_large);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud, *doncloud);

    std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud(cloud);
    don.setNormalScaleLarge(normals_large);
    don.setNormalScaleSmall(normals_small);

    if (!don.initCompute())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature(*doncloud);

    obj.normals = *doncloud;

    // // Build the condition for filtering
    // pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(
    //     new pcl::ConditionOr<pcl::PointNormal>());
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
    //     new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, 0.7)));
    // // Build the filter
    // pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    // condrem.setCondition(range_cond);
    // condrem.setInputCloud(doncloud);

    // pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // // Apply filter
    // condrem.filter(*doncloud_filtered);

    // doncloud = doncloud_filtered;
    // pcl::io::savePCDFile<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test;
    // viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    // viewer_test->setBackgroundColor(0, 0, 0);
    // viewer_test->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, doncloud_filtered, 100, 5, "all", 0);
    // viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(100.0, "World");
    // viewer_test->initCameraParameters();

    // while (!viewer_test->wasStopped())
    // {
    //     viewer_test->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

void calNormalChangesWeight(VoxelizedObj &obj)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

    ne.setRadiusSearch(NORMAL_SEARCH_RADIUS);
    ne.compute(*normals);

    obj.normals = *normals;

    // kd_tree search radius
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    float radius = IMMEDIATE_NEIGHBOUR_RADIUS;

    double largest_weight = 0;
    std::vector<double> point_weight;
    obj.weighting.resize(normals->size());
    for (int i=0; i<normals->size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        obj.weighting.at(i) = 0;
        kdtree.radiusSearch (cloud->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        for(int j=0; j<pointIdxRadiusSearch.size(); j++)
        {
            double individual_weight;
            calNormalDif(normals->at(i), normals->at(pointIdxRadiusSearch.at(j)), individual_weight);
            obj.weighting.at(i) += individual_weight;
        }
        obj.weighting.at(i) = obj.weighting.at(i)/pointIdxRadiusSearch.size();
    }

    // for (int i=0; i<normals->size(); i++)
    // {
    //     obj.weighting.at(i) = normals->at(i).curvature;
    // }

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test;
    // viewer_test.reset(new pcl::visualization::PCLVisualizer("Debugging screen"));
    // viewer_test->setBackgroundColor(0, 0, 0);
    // viewer_test->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, doncloud_filtered, 100, 5, "all", 0);
    // viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(100.0, "World");
    // viewer_test->initCameraParameters();

    // while (!viewer_test->wasStopped())
    // {
    //     viewer_test->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}


void visualizeVoxel(VoxelizedObj &obj, ros::Publisher &publisher)
{
    visualization_msgs::Marker voxelized_object;
    voxelized_object.header.stamp = ros::Time::now();
    voxelized_object.header.frame_id = "map";
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
    for (int i = 0; i < obj.positions.size(); i++)
    {
        voxelized_object.points.at(i).x = obj.positions.at(i).x - 128;
        voxelized_object.points.at(i).y = obj.positions.at(i).y - 128;
        voxelized_object.points.at(i).z = obj.positions.at(i).z;
        voxelized_object.colors.at(i).r = obj.rgbas.at(i).r / 255;
        voxelized_object.colors.at(i).g = obj.rgbas.at(i).g / 255;
        voxelized_object.colors.at(i).b = obj.rgbas.at(i).b / 255;
        voxelized_object.colors.at(i).a = obj.rgbas.at(i).a;
    }

    publisher.publish(voxelized_object);
}

void debugVisualizer(std::vector<pcl::PointXYZ> &points, ros::Publisher &publisher)
{
    visualization_msgs::Marker voxelized_object;
    voxelized_object.header.stamp = ros::Time::now();
    voxelized_object.header.frame_id = "map";
    voxelized_object.type = 5;
    voxelized_object.action = 0;
    // voxelized_object.scale.x = VOXEL_SIZE/2;
    // voxelized_object.scale.y = VOXEL_SIZE/2;
    // voxelized_object.scale.z = VOXEL_SIZE/2;
    voxelized_object.scale.x = 1;
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
        voxelized_object.points.at(i).x = points.at(i).x - 128;
        voxelized_object.points.at(i).y = points.at(i).y - 128;
        voxelized_object.points.at(i).z = points.at(i).z;
        voxelized_object.colors.at(i).r = 1;
        voxelized_object.colors.at(i).g = 0;
        voxelized_object.colors.at(i).b = 0;
        voxelized_object.colors.at(i).a = 1;
    }

    publisher.publish(voxelized_object);
}

void visualizeFOV(VoxelizedObj &obj, std::vector<int>& fov_idx, ros::Publisher &publisher)
{
    visualization_msgs::Marker voxelized_object;
    voxelized_object.header.stamp = ros::Time::now();
    voxelized_object.header.frame_id = "map";
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

    voxelized_object.points.resize(fov_idx.size());
    voxelized_object.colors.resize(fov_idx.size());
    for (int i = 0; i < fov_idx.size(); i++)
    {
        voxelized_object.points.at(i).x = obj.positions.at(fov_idx.at(i)).x - 128;
        voxelized_object.points.at(i).y = obj.positions.at(fov_idx.at(i)).y - 128;
        voxelized_object.points.at(i).z = obj.positions.at(fov_idx.at(i)).z;
        voxelized_object.colors.at(i).r = 0;
        voxelized_object.colors.at(i).g = 1;
        voxelized_object.colors.at(i).b = 0;
        voxelized_object.colors.at(i).a = 1;
    }

    publisher.publish(voxelized_object);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_voxelizer");

    ros::NodeHandle n;

    ros::Publisher publisher = n.advertise<visualization_msgs::Marker>("voxelized_object", 1000);
    ros::Publisher publisher_debug = n.advertise<visualization_msgs::Marker>("debugging", 1000);
    ros::Publisher publisher_fov = n.advertise<visualization_msgs::Marker>("fov", 1000);

    ros::Rate loop_rate(10);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    VoxelizedObj obj;

    mode = 1;
    loadVoxelFile(obj);

    while (ros::ok())
    {
        ROS_INFO("Enter display mode:\n 1. Position Weighting v1\n 2. Position Weighting v2\n 3. DoN Weighting\n 4. Edges\n 5. Combined weight\n 6. Viewpoints");
        std::cin >> mode;
        loadVoxelFile(obj);
        if(mode == 3)
            calDoNWeight(obj);
        if(mode >= 4)
            calNormalChangesWeight(obj);
        chooseColorVox(obj);
        visualizeVoxel(obj, publisher);

        pcl::PointXYZ sample_view;

        GenViewpoint viewpoint_gen(obj.positions.at(180237),obj.normals.at(180237));
        // GenViewpoint viewpoint_gen(obj.positions.at(402987),obj.normals.at(402987));
        viewpoint_gen.getConstraints(0, 1, 0, 30);
        viewpoint_gen.calViewpointLoc(sample_view);

        std::vector<pcl::PointXYZ> points;
        points.resize(2);
        points.at(0) = obj.positions.at(180237);
        // points.at(0) = obj.positions.at(402987);
        points.at(1) = sample_view;
        debugVisualizer(points,publisher_debug);
        std::cout << sample_view.x << " " << sample_view.y << " " << sample_view.z << std::endl;
        std::cout << obj.positions.at(180237) << " - " << obj.normals.at(180237).normal_x << " " << obj.normals.at(180237).normal_y << " " << obj.normals.at(180237).normal_z << std::endl;

        std::vector<int> fov_set_idx;
        FOVCheck fov_check(obj.positions, 180237, sample_view);
        fov_check.initiateFOVCheck(fov_set_idx);
        std::cout << "FOV size: " << fov_set_idx.size() << std::endl;
        visualizeFOV(obj, fov_set_idx, publisher_fov);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}