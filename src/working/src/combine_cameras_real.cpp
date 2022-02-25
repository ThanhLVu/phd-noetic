#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <stdint.h>
#include <vector>

const double CAMERA_MAX_RANGE = 1;
const float VOXEL_GRID_SIZE = 0.01;
const int MAXIMUM_NO_ITERATIONS = 20;
struct singleCluster
{
    int unique_index;
    pcl::PointIndices points_indices;
    pcl::PointCloud<pcl::PointXYZRGB> points_data;
    std::vector<pcl::PointXYZ> bounding_box;
};

ros::Publisher markers_pub;

geometry_msgs::TransformStamped transformStamped;

pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
pcl::PCLPointCloud2 cloud_downsampled;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_cam_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_cam_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_cam_cloud_relative(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_removed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall_removed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::ModelCoefficients::Ptr coefficients_ptr(new pcl::ModelCoefficients());

std::vector<singleCluster> single_cluster_vec;

// Call back function left camera
void leftCamCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Downsampling input point cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    sor.filter(cloud_downsampled);

    pcl::fromPCLPointCloud2(cloud_downsampled, *left_cam_cloud);

    /**************************Filter by range*************************/
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(left_cam_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, CAMERA_MAX_RANGE);
    pass.filter(*left_cam_cloud);

    //     boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test_left;
    //     viewer_test_left.reset(new pcl::visualization::PCLVisualizer("Debugging screen left"));
    //     viewer_test_left->setBackgroundColor(0, 0, 0);
    //     viewer_test_left->addPointCloud(left_cam_cloud, "all");
    //     viewer_test_left->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    //     viewer_test_left->initCameraParameters();

    //     while (!viewer_test_left->wasStopped()) {
    //         viewer_test_left->spinOnce(100);
    //         ros::Duration(0.1).sleep();
    //     }

    // ROS_INFO("Received left point cloud");
}

// Call back function left camera
void rightCamCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Downsampling input point cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    sor.filter(cloud_downsampled);

    pcl::fromPCLPointCloud2(cloud_downsampled, *right_cam_cloud);

    /**************************Filter by range*************************/
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(right_cam_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, CAMERA_MAX_RANGE);
    pass.filter(*right_cam_cloud);

    //     boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test_right;
    //     viewer_test_right.reset(new pcl::visualization::PCLVisualizer("Debugging screen right"));
    //     viewer_test_right->setBackgroundColor(0, 0, 0);
    //     viewer_test_right->addPointCloud(right_cam_cloud, "all");
    //     viewer_test_right->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    //     viewer_test_right->initCameraParameters();

    //     while (!viewer_test_right->wasStopped()) {
    //         viewer_test_right->spinOnce(100);
    //         ros::Duration(0.1).sleep();
    //     }

    // ROS_INFO("Received right point cloud");
}

// Obtain transformation matrix from tf tree
Eigen::Matrix4d getTransformationMatrix()
{
    Eigen::Vector3d translation(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    Eigen::Quaterniond rotationQuaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    Eigen::Matrix3d rotationMatrix = rotationQuaternion.toRotationMatrix();
    Eigen::Matrix4d transformationMatrix;

    transformationMatrix << rotationMatrix, translation,
        0.0, 0.0, 0.0, 1.0;

    return transformationMatrix;
}

// Generate the left point cloud relative to the right camera frame
void generateRelativeCloud(Eigen::Matrix4d transform)
{
    left_cam_cloud_relative->clear();

    pcl::PointXYZRGB processing_point;
    Eigen::Vector4d relative_position, original_position;
    for (int i = 0; i < left_cam_cloud->size(); i++)
    {
        original_position << left_cam_cloud->points.at(i).x, left_cam_cloud->points.at(i).y, left_cam_cloud->points.at(i).z, 1.0;
        relative_position = transform * original_position;

        processing_point.x = relative_position[0];
        processing_point.y = relative_position[1];
        processing_point.z = relative_position[2];
        processing_point.rgb = left_cam_cloud->points.at(i).rgb;

        left_cam_cloud_relative->push_back(processing_point);
    }

    //     boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test_relative;
    //     viewer_test_relative.reset(new pcl::visualization::PCLVisualizer("Debugging screen relative"));
    //     viewer_test_relative->setBackgroundColor(0, 0, 0);
    //     viewer_test_relative->addPointCloud(right_cam_cloud, "all");
    //     viewer_test_relative->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    //     viewer_test_relative->initCameraParameters();

    //     while (!viewer_test_relative->wasStopped()) {
    //         viewer_test_relative->spinOnce(100);
    //         ros::Duration(0.1).sleep();
    //     }
}

// Combine two point clouds together
void combinePointclouds()
{
    if (left_cam_cloud->size() <= 0 && right_cam_cloud->size() <= 0)
    {
        return;
        ROS_INFO("Camera point clouds are empty");
        std::cout << "Left camera cloud size: " << left_cam_cloud->size() << endl;
        std::cout << "Right camera cloud size: " << right_cam_cloud->size() << endl;
    }

    Eigen::Matrix4d transform = getTransformationMatrix();
    ROS_INFO("Got transformation matrix between cameras");

    generateRelativeCloud(transform);
    ROS_INFO("Generated relative point clouds");

    // Generate combined point cloud
    combined_cloud = right_cam_cloud;
    for (int i = 0; i < left_cam_cloud_relative->size(); i++)
    {
        combined_cloud->push_back(left_cam_cloud_relative->points.at(i));
    }
    ROS_INFO("Generated combined point clouds");

    // Downsampling combined point cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(combined_cloud);
    voxel.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    voxel.filter(*combined_cloud);

    ROS_INFO("Generated downsized combined point clouds");

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_combined;
    // viewer_test_combined.reset(new pcl::visualization::PCLVisualizer("Debugging screen combined"));
    // viewer_test_combined->setBackgroundColor(0, 0, 0);
    // viewer_test_combined->addPointCloud(combined_cloud, "all");
    // viewer_test_combined->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // //    viewer_test->addCoordinateSystem(1.0, "World");
    // viewer_test_combined->initCameraParameters();

    // while (!viewer_test_combined->wasStopped())
    // {
    //     viewer_test_combined->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

// Ground Plane Removal Function
void removeGround()
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.03);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(combined_cloud);
    seg.segment(*inliers, *coefficients_ptr);

    // Extract the inliers
    extract.setInputCloud(combined_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*ground_removed_cloud);
}

// Find projection of Origin (0,0,0) on RANSAC generated surface
std::vector<float> projectOrigin(pcl::ModelCoefficients coefficients)
// Assumption: coefficients(a,b,c,d)
{
    float a = coefficients.values.at(0);
    float b = coefficients.values.at(1);
    float c = coefficients.values.at(2);
    float d = coefficients.values.at(3);

    float t = -d / (pow(a, 2.0) + pow(b, 2.0) + pow(c, 2.0));

    std::vector<float> projection;
    projection.resize(3);
    projection.at(0) = a * t;
    projection.at(1) = b * t;
    projection.at(2) = c * t;

    return projection;
}

// Locate points behind surface generated by RANSAC
pcl::PointIndices getPointBehindIndices(std::vector<float> origin_projection)
// Assumption: origin_projection(x,y,z)
{
    Eigen::Vector3f toOriginVec, toPointVec;
    pcl::PointIndices point_behind_indices;

    point_behind_indices.indices.clear();

    toOriginVec(0) = -origin_projection.at(0);
    toOriginVec(1) = -origin_projection.at(1);
    toOriginVec(2) = -origin_projection.at(2);

    for (int i = 0; i < ground_removed_cloud->points.size(); i++)
    {
        toPointVec(0) = ground_removed_cloud->points.at(i).x - origin_projection.at(0);
        toPointVec(1) = ground_removed_cloud->points.at(i).y - origin_projection.at(1);
        toPointVec(2) = ground_removed_cloud->points.at(i).z - origin_projection.at(2);

        if (toOriginVec.dot(toPointVec) < 0)
            point_behind_indices.indices.push_back(i);
    }

    std::cout << "Got " << point_behind_indices.indices.size() << " points behind RANSAC generated plane and "
              << ground_removed_cloud->points.size() - point_behind_indices.indices.size() << " points infront" << std::endl;

    return point_behind_indices;
}

// Remove background (Everything below ground level)
void removeBackground()
{
    pcl::PointIndices::Ptr point_behind_indices(new pcl::PointIndices());
    *point_behind_indices = getPointBehindIndices(projectOrigin(*coefficients_ptr));

    pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr indices_extract_(new pcl::ExtractIndices<pcl::PointXYZRGB>);
    indices_extract_->setInputCloud(ground_removed_cloud);
    indices_extract_->setIndices(point_behind_indices);
    indices_extract_->setNegative(true);
    indices_extract_->filter(*background_removed_cloud);

    // boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test_ground_removed;
    // viewer_test_ground_removed.reset(new pcl::visualization::PCLVisualizer("Debugging screen ground removed cloud"));
    // viewer_test_ground_removed->setBackgroundColor(0, 0, 0);
    // viewer_test_ground_removed->addPointCloud(background_removed_cloud, "all");
    // viewer_test_ground_removed->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // viewer_test_ground_removed->initCameraParameters();

    // while (!viewer_test_ground_removed->wasStopped()) {
    //     viewer_test_ground_removed->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

// Remove wall, leave exposed objects (removeGround + removeBackground again)
void removeWall()
{
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations (1000);
    // seg.setDistanceThreshold (0.01);

    // // Create the filtering object
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // // Segment the largest planar component from the remaining cloud
    // seg.setInputCloud (background_removed_cloud);
    // seg.segment (*inliers, *coefficients_ptr);

    // // Extract the inliers
    // extract.setInputCloud (background_removed_cloud);
    // extract.setIndices (inliers);
    // extract.setNegative (true);
    // extract.filter (*wall_removed_cloud);

    wall_removed_cloud = background_removed_cloud;
}

// Euclidean clustering
std::vector<pcl::PointIndices> euclideanClustering()
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(combined_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices.clear();

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.03); // 1cm
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(wall_removed_cloud);
    ec.extract(cluster_indices);

    // Storing the clustered group indices
    single_cluster_vec.resize(cluster_indices.size());
    for (int i = 0; i < single_cluster_vec.size(); i++)
    {
        single_cluster_vec.at(i).unique_index = i;
        single_cluster_vec.at(i).points_indices = cluster_indices.at(i);
    }

    std::cout << "I clustered " << cluster_indices.size() << " set from " << wall_removed_cloud->points.size() << " points" << std::endl;
    return cluster_indices;
}

// Generate the cloud with multiple clusters
void generateClusteredCloud(std::vector<pcl::PointIndices> &cluster_indices)
{
    clustered_cloud->clear();

    for (int unique_index = 0; unique_index < single_cluster_vec.size(); unique_index++)
    {
        for (int point_index = 0; point_index < single_cluster_vec.at(unique_index).points_indices.indices.size(); point_index++)
        {
            single_cluster_vec.at(unique_index).points_data.push_back(wall_removed_cloud->at(single_cluster_vec.at(unique_index).points_indices.indices.at(point_index)));
            clustered_cloud->push_back(wall_removed_cloud->at(single_cluster_vec.at(unique_index).points_indices.indices.at(point_index)));
        }
        std::cout << "Cloud after stored into struct: " << single_cluster_vec.at(unique_index).points_data.points.size() << std::endl;
    }

    std::cout << "Cloud after background and wall removed: " << wall_removed_cloud->points.size() << std::endl;
    std::cout << "Cloud after clustered: " << clustered_cloud->points.size() << std::endl;

    // boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test_clustered_cloud;
    // viewer_test_clustered_cloud.reset(new pcl::visualization::PCLVisualizer("Debugging screen clustered clous"));
    // viewer_test_clustered_cloud->setBackgroundColor(0, 0, 0);
    // viewer_test_clustered_cloud->addPointCloud(clustered_cloud, "all");
    // viewer_test_clustered_cloud->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    // viewer_test_clustered_cloud->initCameraParameters();

    // while (!viewer_test_clustered_cloud->wasStopped()) {
    //     viewer_test_clustered_cloud->spinOnce(100);
    //     ros::Duration(0.1).sleep();
    // }
}

// Convert the min max points from PCA frame to World frame
void convertBoxToWorldFrame(Eigen::Matrix3f &rotation, Eigen::Vector4f &centroid, std::vector<pcl::PointXYZ> &original_set, std::vector<pcl::PointXYZ> &converted_set)
{
    converted_set.resize(original_set.size());

    Eigen::Matrix4f transformationMatrix;
    transformationMatrix = Eigen::Matrix4f::Zero();
    transformationMatrix.block<3, 3>(0, 0) = rotation;
    transformationMatrix.block<4, 1>(0, 3) = centroid;

    for (int i = 0; i < original_set.size(); i++)
    {
        Eigen::Vector4f ori_point_vec;
        ori_point_vec << original_set.at(i).x, original_set.at(i).y, original_set.at(i).z, 1.0;

        Eigen::Vector4f converted_point_vec = transformationMatrix * ori_point_vec;

        converted_set.at(i).x = converted_point_vec(0);
        converted_set.at(i).y = converted_point_vec(1);
        converted_set.at(i).z = converted_point_vec(2);
    }
}

// Use PCA to transfer the clusters into individual coordinate system
void pcaClusterBox(std::vector<singleCluster> &single_cluster)
{
    pcl::PCA<pcl::PointXYZRGB> pca;

    for (int unique_index = 0; unique_index < single_cluster.size(); unique_index++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr individual_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        *individual_cluster = single_cluster.at(unique_index).points_data;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_individual_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        pca.setInputCloud(individual_cluster);
        pca.project(*individual_cluster, *projected_individual_cluster);

        Eigen::Vector4f min_pt, max_pt;

        pcl::getMinMax3D(*projected_individual_cluster, min_pt, max_pt);
        double max_x, max_y, max_z, min_x, min_y, min_z;
        max_x = max_pt(0);
        max_y = max_pt(1);
        max_z = max_pt(2);
        min_x = min_pt(0);
        min_y = min_pt(1);
        min_z = min_pt(2);

        single_cluster.at(unique_index).bounding_box.resize(8);
        pcl::PointXYZ corner_1(min_x, min_y, min_z);
        single_cluster.at(unique_index).bounding_box.at(0) = corner_1;
        pcl::PointXYZ corner_2(min_x, min_y, max_z);
        single_cluster.at(unique_index).bounding_box.at(1) = corner_2;
        pcl::PointXYZ corner_3(min_x, max_y, min_z);
        single_cluster.at(unique_index).bounding_box.at(2) = corner_3;
        pcl::PointXYZ corner_4(min_x, max_y, max_z);
        single_cluster.at(unique_index).bounding_box.at(3) = corner_4;
        pcl::PointXYZ corner_5(max_x, min_y, min_z);
        single_cluster.at(unique_index).bounding_box.at(4) = corner_5;
        pcl::PointXYZ corner_6(max_x, min_y, max_z);
        single_cluster.at(unique_index).bounding_box.at(5) = corner_6;
        pcl::PointXYZ corner_7(max_x, max_y, min_z);
        single_cluster.at(unique_index).bounding_box.at(6) = corner_7;
        pcl::PointXYZ corner_8(max_x, max_y, max_z);
        single_cluster.at(unique_index).bounding_box.at(7) = corner_8;

        // boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer_test_bounding_box;
        // viewer_test_bounding_box.reset(new pcl::visualization::PCLVisualizer("Debugging screen bounding box" + std::to_string(unique_index)));
        // viewer_test_bounding_box->setBackgroundColor(0, 0, 255);
        // viewer_test_bounding_box->addPointCloud(projected_individual_cluster, "all");
        // viewer_test_bounding_box->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
        // viewer_test_bounding_box->initCameraParameters();

        // // Connect corner 1 to corners 2, 3, 5
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(1), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(0));
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(2), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(1));
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(4), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(2));
        // // Connect corner 2 to corners 4, 6
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(1), single_cluster.at(unique_index).bounding_box.at(3), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(3));
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(1), single_cluster.at(unique_index).bounding_box.at(5), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(4));
        // // Connect corner 3 to corners 4, 7
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(2), single_cluster.at(unique_index).bounding_box.at(3), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(5));
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(2), single_cluster.at(unique_index).bounding_box.at(6), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(6));
        // // Connect corner 4 to 8
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(3), single_cluster.at(unique_index).bounding_box.at(7), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(7));
        // // Connect corner 5 to 6, 7
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(4), single_cluster.at(unique_index).bounding_box.at(5), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(8));
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(4), single_cluster.at(unique_index).bounding_box.at(6), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(9));
        // // Connect 6 to 8
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(5), single_cluster.at(unique_index).bounding_box.at(7), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(10));
        // // Connect 7 to 8
        // viewer_test_bounding_box->addLine(single_cluster.at(unique_index).bounding_box.at(6), single_cluster.at(unique_index).bounding_box.at(7), 255.0, 0.0, 0.0, std::to_string(unique_index)+"line"+std::to_string(11));

        // while (!viewer_test_bounding_box->wasStopped()) {
        //     viewer_test_bounding_box->spinOnce(100);
        //     ros::Duration(0.1).sleep();
        // }

        std::vector<pcl::PointXYZ> converted_box;
        convertBoxToWorldFrame(pca.getEigenVectors(), pca.getMean(), single_cluster.at(unique_index).bounding_box, converted_box);

        single_cluster.at(unique_index).bounding_box = converted_box;
    }
}

// Visualize the bounding in world frame
void drawWorldView(std::vector<singleCluster> &single_cluster)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test_bounding_box_world;
    viewer_test_bounding_box_world.reset(new pcl::visualization::PCLVisualizer("Debugging screen bounding box_world"));
    viewer_test_bounding_box_world->setBackgroundColor(255, 255, 255);
    viewer_test_bounding_box_world->addPointCloud(wall_removed_cloud, "all");
    viewer_test_bounding_box_world->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
    viewer_test_bounding_box_world->initCameraParameters();

    for (int unique_index = 0; unique_index < single_cluster.size(); unique_index++)
    {

        // Connect corner 1 to corners 2, 3, 5
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(1), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(0));
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(2), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(1));
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(4), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(2));
        // Connect corner 2 to corners 4, 6
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(1), single_cluster.at(unique_index).bounding_box.at(3), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(3));
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(1), single_cluster.at(unique_index).bounding_box.at(5), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(4));
        // Connect corner 3 to corners 4, 7
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(2), single_cluster.at(unique_index).bounding_box.at(3), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(5));
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(2), single_cluster.at(unique_index).bounding_box.at(6), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(6));
        // Connect corner 4 to 8
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(3), single_cluster.at(unique_index).bounding_box.at(7), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(7));
        // Connect corner 5 to 6, 7
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(4), single_cluster.at(unique_index).bounding_box.at(5), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(8));
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(4), single_cluster.at(unique_index).bounding_box.at(6), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(9));
        // Connect 6 to 8
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(5), single_cluster.at(unique_index).bounding_box.at(7), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(10));
        // Connect 7 to 8
        viewer_test_bounding_box_world->addLine(single_cluster.at(unique_index).bounding_box.at(6), single_cluster.at(unique_index).bounding_box.at(7), 255.0, 0.0, 0.0, std::to_string(unique_index) + "line" + std::to_string(11));
    }

    while (!viewer_test_bounding_box_world->wasStopped())
    {
        viewer_test_bounding_box_world->spinOnce(100);
        ros::Duration(0.1).sleep();
    }
}

// Add point to visualization point list
void addPointToList(pcl::PointXYZ &input1, pcl::PointXYZ &input2, std::vector<geometry_msgs::Point> &point_list)
{
    geometry_msgs::Point point1, point2;
    point1.x = input1.x;
    point1.y = input1.y;
    point1.z = input1.z;

    point2.x = input2.x;
    point2.y = input2.y;
    point2.z = input2.z;

    point_list.push_back(point1);
    point_list.push_back(point2);
}

// Publish bounding boxes as RvizMarkers msg
void publishMarkersMsg(std::vector<singleCluster> &single_cluster)
{
    visualization_msgs::Marker visualization_boxes;
    visualization_boxes.header.frame_id = "camera2_depth_optical_frame";
    visualization_boxes.header.stamp = ros::Time::now();
    visualization_boxes.lifetime = ros::Duration(1.0);
    visualization_boxes.action = visualization_msgs::Marker::ADD;
    visualization_boxes.pose.orientation.w = 1.0;
    visualization_boxes.type = visualization_msgs::Marker::LINE_LIST;
    visualization_boxes.scale.x = 0.001;
    visualization_boxes.color.a = 1.0;
    visualization_boxes.color.r = 1.0;

    for (int unique_index = 0; unique_index < single_cluster.size(); unique_index++)
    {
        // Connect corner 1 to corners 2, 3, 5
        addPointToList(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(1), visualization_boxes.points);
        addPointToList(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(2), visualization_boxes.points);
        addPointToList(single_cluster.at(unique_index).bounding_box.at(0), single_cluster.at(unique_index).bounding_box.at(4), visualization_boxes.points);
        // Connect corner 2 to corners 4, 6
        addPointToList(single_cluster.at(unique_index).bounding_box.at(1), single_cluster.at(unique_index).bounding_box.at(3), visualization_boxes.points);
        addPointToList(single_cluster.at(unique_index).bounding_box.at(1), single_cluster.at(unique_index).bounding_box.at(5), visualization_boxes.points);
        // Connect corner 3 to corners 4, 7
        addPointToList(single_cluster.at(unique_index).bounding_box.at(2), single_cluster.at(unique_index).bounding_box.at(3), visualization_boxes.points);
        addPointToList(single_cluster.at(unique_index).bounding_box.at(2), single_cluster.at(unique_index).bounding_box.at(6), visualization_boxes.points);
        // Connect corner 4 to 8
        addPointToList(single_cluster.at(unique_index).bounding_box.at(3), single_cluster.at(unique_index).bounding_box.at(7), visualization_boxes.points);
        // Connect corner 5 to 6, 7
        addPointToList(single_cluster.at(unique_index).bounding_box.at(4), single_cluster.at(unique_index).bounding_box.at(5), visualization_boxes.points);
        addPointToList(single_cluster.at(unique_index).bounding_box.at(4), single_cluster.at(unique_index).bounding_box.at(6), visualization_boxes.points);
        // Connect 6 to 8
        addPointToList(single_cluster.at(unique_index).bounding_box.at(5), single_cluster.at(unique_index).bounding_box.at(7), visualization_boxes.points);
        // Connect 7 to 8
        addPointToList(single_cluster.at(unique_index).bounding_box.at(6), single_cluster.at(unique_index).bounding_box.at(7), visualization_boxes.points);
    }

    markers_pub.publish(visualization_boxes);
}

// Finding bounding box procedure
void cloudProcess()
{
    single_cluster_vec.clear();

    removeGround();
    removeBackground();
    removeWall();

    std::vector<pcl::PointIndices> cluster_indices = euclideanClustering();
    generateClusteredCloud(cluster_indices);
    pcaClusterBox(single_cluster_vec);
    // drawWorldView(single_cluster_vec);
    publishMarkersMsg(single_cluster_vec);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combine_cameras_real");

    // NodeHandle
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);

    // Get point cloud data of each camera - Using 1 camera for now
    ros::Subscriber left_pointcloud_sub = nh.subscribe("/camera1/depth/color/points", 1, leftCamCallback);
    ros::Subscriber right_pointcloud_sub = nh.subscribe("/camera2/depth/color/points", 1, rightCamCallback);

    // Publish rviz marker topic
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_markers", 0);

    while (ros::ok())
    {
        // Get transform from left camera to right camera  (from the urdf model)
        try
        {
            // Change link name if needed
            transformStamped = tfBuffer.lookupTransform("camera2_depth_optical_frame", "camera1_depth_optical_frame", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ROS_INFO("Combining two cameras' point clouds");
        combinePointclouds();

        if (!combined_cloud->points.empty())
        {
            ROS_INFO("Clustering point cloud into groups");
            cloudProcess();
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
