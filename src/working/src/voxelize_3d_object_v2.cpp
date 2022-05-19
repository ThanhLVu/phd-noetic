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
#include "fov_check_v2.h"

#include <iostream>
#include <string>
#include <random>
#include <Eigen/Dense>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OccupancyOcTreeBase.h>

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
const double BUCKET_BOUND_X_MAX = 35;
const double BUCKET_BOUND_X_MIN = -91;
const double BUCKET_BOUND_Y_MAX = 68;
const double BUCKET_BOUND_Y_MIN = -68;
const double BUCKET_BOUND_Z_MAX = 58;
const double BUCKET_BOUND_Z_MIN = 14;
const double NORMAL_SEARCH_RADIUS = 4;

// Load magicavoxel file
void loadVoxelFile(VoxelizedObj &obj)
{
    magicavoxel::VoxFile vox_file(true, true);
    vox_file.Load(VOXEL_FILE);
    cout << "There are " << vox_file.sparseModels().at(0).voxels().size() << " voxels in the original model." << endl;
    magicavoxel::VoxSparseModel sparse_model = vox_file.sparseModels().at(0);

    obj.positions.resize(sparse_model.voxels().size());

    for (int i = 0; i < sparse_model.voxels().size(); i++)
    {
        obj.positions.at(i).x = (unsigned short)sparse_model.voxels().at(i).x - 110 - 0.5;
        obj.positions.at(i).y = (unsigned short)sparse_model.voxels().at(i).y - 128 - 0.5;
        obj.positions.at(i).z = (unsigned short)sparse_model.voxels().at(i).z - 56 - 0.5;
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

void weightRegistration(VoxelizedObj &obj, pcl::PointIndices &above_bucket_idx)
{
    above_bucket_idx.indices.clear();
    obj.weighting.resize(obj.positions.size());
    
    // Anything within xy bound and above z bound of the bucket are 1, everything else is 0
    for(int i=0; i<obj.positions.size(); i++)
    {
        if(obj.positions.at(i).x > BUCKET_BOUND_X_MIN && obj.positions.at(i).x < BUCKET_BOUND_X_MAX
        && obj.positions.at(i).y > BUCKET_BOUND_Y_MIN && obj.positions.at(i).y < BUCKET_BOUND_Y_MAX
        && obj.positions.at(i).z > BUCKET_BOUND_Z_MAX)          
        {
            obj.weighting.at(i) = 1;
            above_bucket_idx.indices.push_back(i);
        }     
        else obj.weighting.at(i) = 0;
    }
    cout << "There are " << above_bucket_idx.indices.size() << " voxels above truck bucket." << endl;
}

void chooseColorVox(VoxelizedObj &obj)
{
    obj.rgbas.resize(obj.positions.size());  

    // Anything within bucket bound and above the bucket are colored white
    for (int i = 0; i < obj.positions.size(); i++)
    {
        if(obj.weighting.at(i) == 1)      
        {
            obj.rgbas.at(i).r = 255;
            obj.rgbas.at(i).b = 255;
            obj.rgbas.at(i).g = 255;
            obj.rgbas.at(i).a = 1;
        }     
        else 
        {
            obj.rgbas.at(i).r = 0;
            obj.rgbas.at(i).b = 0;
            obj.rgbas.at(i).g = 0;
            obj.rgbas.at(i).a = 1;
        }
    }
}

void calVoxelNormals(VoxelizedObj &obj)
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
    // ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    ne.setViewPoint(0, 0, 0);

    // calculate normals
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(NORMAL_SEARCH_RADIUS);
    ne.compute(*normals);

    obj.normals = *normals;
}

void findFOVCenterVoxel(VoxelizedObj &obj, pcl::PointIndices &above_bucket_points_idx, pcl::PointNormal &viewpoint, int &fov_center_point_idx)
{
    double min_dif = 1;
    int center_point_idx;
    for(int i=0; i<above_bucket_points_idx.indices.size(); i++)
    {
        // Calculate dot product
        Eigen::Vector3d viewpoint_to_voxel_center(obj.positions.at(above_bucket_points_idx.indices.at(i)).x - viewpoint.x,
                                                  obj.positions.at(above_bucket_points_idx.indices.at(i)).y - viewpoint.y,
                                                  obj.positions.at(above_bucket_points_idx.indices.at(i)).z - viewpoint.z);
        Eigen::Vector3d viewpoint_direction(viewpoint.normal_x, viewpoint.normal_y, viewpoint.normal_z);
        Eigen::Vector3d current_point_normal(obj.normals.at(above_bucket_points_idx.indices.at(i)).normal_x,
                                             obj.normals.at(above_bucket_points_idx.indices.at(i)).normal_y,
                                             obj.normals.at(above_bucket_points_idx.indices.at(i)).normal_z);
        double dot_product = viewpoint_direction.dot(viewpoint_to_voxel_center)/(viewpoint_direction.norm() * viewpoint_to_voxel_center.norm());
        double dot_product_normal_check = current_point_normal.dot(viewpoint_direction)/(current_point_normal.norm() * viewpoint_direction.norm());
        // Looking for the point with dot product cloest to 1 and the point normal is not opposite of the viewpoint direction
        // if(dot_product > 0 && fabs(dot_product - 1) < min_dif && dot_product_normal_check > 0)
        if(dot_product > 0 && fabs(dot_product - 1) < min_dif)
        {
            center_point_idx = i;
            min_dif = fabs(dot_product - 1);
        }
    }
    fov_center_point_idx = above_bucket_points_idx.indices.at(center_point_idx);
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
        voxelized_object.points.at(i).x = obj.positions.at(i).x;
        voxelized_object.points.at(i).y = obj.positions.at(i).y;
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
        voxelized_object.points.at(i).x = points.at(i).x;
        voxelized_object.points.at(i).y = points.at(i).y;
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
        voxelized_object.points.at(i).x = obj.positions.at(fov_idx.at(i)).x;
        voxelized_object.points.at(i).y = obj.positions.at(fov_idx.at(i)).y;
        voxelized_object.points.at(i).z = obj.positions.at(fov_idx.at(i)).z;
        voxelized_object.colors.at(i).r = 0;
        voxelized_object.colors.at(i).g = 1;
        voxelized_object.colors.at(i).b = 0;
        voxelized_object.colors.at(i).a = 1;
    }

    publisher.publish(voxelized_object);
}

void removeDup(std::vector<int> &vec)
{
    auto end = vec.end();
    for (auto it = vec.begin(); it != end; ++it) {
        end = std::remove(it + 1, end, *it);
    }
 
    vec.erase(end, vec.end());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_voxelizer");

    ros::NodeHandle n;

    ros::Publisher publisher = n.advertise<visualization_msgs::Marker>("voxelized_object", 1000);
    ros::Publisher publisher_debug = n.advertise<visualization_msgs::Marker>("debugging", 1000);
    ros::Publisher publisher_fov = n.advertise<visualization_msgs::Marker>("fov", 1000);

    ros::Rate loop_rate(10);
    pcl::PointIndices above_bucket_points_idx;
    VoxelizedObj obj;

    while (ros::ok())
    {
        loadVoxelFile(obj);
        ROS_INFO("Finished loading voxel file.");
        calVoxelNormals(obj);
        ROS_INFO("Finished calculating normals vectors");
        weightRegistration(obj, above_bucket_points_idx);
        ROS_INFO("Finished registering voxel weights.");        
        chooseColorVox(obj);
        ROS_INFO("Finished coloring the model.");
        visualizeVoxel(obj, publisher);
        ROS_INFO("Finished loading model into rviz.");

        // Viewpoint 1
        pcl::PointNormal viewpoint1;
        viewpoint1.x = 39.5;
        viewpoint1.y = 9.5;
        viewpoint1.z = 109.5;
        viewpoint1.normal_x = -1;
        viewpoint1.normal_y = 0;
        viewpoint1.normal_z = -1;

        int fov1_center_point_idx;
        pcl::PointXYZ fov1_center_point;
        findFOVCenterVoxel(obj, above_bucket_points_idx, viewpoint1, fov1_center_point_idx);
        fov1_center_point = obj.positions.at(fov1_center_point_idx);
        ROS_INFO("Finished calculating the fov center point");
        
        std::vector<pcl::PointXYZ> debug_input;
        debug_input.clear();
        pcl::PointXYZ viewpoint1_pos(viewpoint1.x, viewpoint1.y, viewpoint1.z);
        pcl::PointXYZ direction_visualizer(viewpoint1.x + 10 * viewpoint1.normal_x,
                                           viewpoint1.y + 10 * viewpoint1.normal_y, 
                                           viewpoint1.z + 10 * viewpoint1.normal_z);
        debug_input.push_back(viewpoint1_pos);
        debug_input.push_back(fov1_center_point);
        debug_input.push_back(viewpoint1_pos);
        debug_input.push_back(direction_visualizer);
        debugVisualizer(debug_input, publisher_debug);
        ROS_INFO("Finished visualize debug information.");

        FOVCheckV2 fov1_checker(obj.positions, obj.normals, fov1_center_point_idx, viewpoint1);
        std::vector<int> fov1_points_idx;
        fov1_checker.initiateFOVCheck(fov1_points_idx);
        visualizeFOV(obj, fov1_points_idx, publisher_fov);
        ROS_INFO("Finished finding the fov and visualize them in Rviz");

        // Viewpoint 2
        pcl::PointNormal viewpoint2;
        viewpoint2.x = -89.5;
        viewpoint2.y = 9.5;
        viewpoint2.z = 109.5;
        viewpoint2.normal_x = 1;
        viewpoint2.normal_y = 0;
        viewpoint2.normal_z = -2;

        int fov2_center_point_idx;
        pcl::PointXYZ fov2_center_point;
        findFOVCenterVoxel(obj, above_bucket_points_idx, viewpoint2, fov2_center_point_idx);
        fov2_center_point = obj.positions.at(fov2_center_point_idx);
        ROS_INFO("Finished calculating the fov center point");

        pcl::PointXYZ viewpoint2_pos(viewpoint2.x, viewpoint2.y, viewpoint2.z);
        pcl::PointXYZ direction_visualizer2(viewpoint2.x + 10 * viewpoint2.normal_x,
                                           viewpoint2.y + 10 * viewpoint2.normal_y, 
                                           viewpoint2.z + 10 * viewpoint2.normal_z);

        debug_input.push_back(viewpoint2_pos);
        debug_input.push_back(fov2_center_point);
        debug_input.push_back(viewpoint2_pos);
        debug_input.push_back(direction_visualizer2);
        debugVisualizer(debug_input, publisher_debug);
        ROS_INFO("Finished visualize debug information.");

        FOVCheckV2 fov2_checker(obj.positions, obj.normals, fov2_center_point_idx, viewpoint2);
        std::vector<int> fov2_points_idx;
        fov2_checker.initiateFOVCheck(fov2_points_idx);
        visualizeFOV(obj, fov2_points_idx, publisher_fov);
        ROS_INFO("Finished finding the fov and visualize them in Rviz");

        std::vector<int> temp_var;
        temp_var.insert(temp_var.begin(), fov1_points_idx.begin(), fov1_points_idx.end());
        temp_var.insert(temp_var.begin(), fov2_points_idx.begin(), fov2_points_idx.end());
        removeDup(temp_var);
        // visualizeFOV(obj, temp_var, publisher_fov);
        temp_var.insert(temp_var.begin(), above_bucket_points_idx.indices.begin(), above_bucket_points_idx.indices.end());
        int temp_var_merged_size = temp_var.size();
        removeDup(temp_var);
        int no_fov_points_above_bucket = temp_var_merged_size - temp_var.size();
        float coverage_percentage = (float)no_fov_points_above_bucket / (float)temp_var_merged_size * 100;
        std::cout << "The sensor covered " << coverage_percentage << " percent of the load" << std::endl;

        ROS_INFO("Finished one loop. Press enter to continue");
        std::cin.ignore();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}