#ifndef EXPANDSEARCH_H
#define EXPANDSEARCH_H

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/visualization/cloud_viewer.h"

#include <vector>
#include <stdlib.h>

/*!
*   \ingroup    ac_expandsearch ExpandSearch
*   \brief      ExpandSearch class
*   \details    This class is for iterate through set of points.\n
*/
class ExpandSearch
{
public:
    /*!
    *   ExpandSearch class constructor
    */
    ExpandSearch(pcl::PointCloud<pcl::PointXYZ>, double);

protected:
    double searching_radius_; // The radius which radiusSearch is conducted with
    int processing_point_index_; // Index of the point which radiusSearch is conducted with
    bool new_cluster_; // Flag if new cluster created
    uint16_t current_cluster_color_; // Color of current cluster

    std::vector<int> neighbor_indices_vector_; // Indices of neighbors received from radiusSearch
    std::vector<int> neighbor_indices_vector_unique_; // Unique indices of neighbors received from radiusSearch
    std::vector<float> neighbor_distance_vector_; // Distance from processing point to neighbors received from radiusSearch

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_; // Pointer for input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr processing_cloud_; // Pointer for input cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_; // Pointer for input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_neighbor_cloud_; // Temporary set of neighbor points


    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree_; // Create class to implement kd tree virtual functions
//    pcl::ExtractIndices<pcl::PointXYZ>::Ptr indices_extract_; // Create class to extract multiple points from point cloud

    /*!
    *   Get input cloud
    */
    void getInputCloud(pcl::PointCloud<pcl::PointXYZ>);

    /*!
    *   Get input cloud
    */
    void getVoxelGridSize(double);

    /*!
    *   Generate random first point index
    */
    int generateRandomNumber();

    /*!
    *   Randomize the color code for the new cluster
    */
    uint32_t randomizeColorCode();

    /*!
    *   Move processing point to clustered point cloud
    */
    void moveProcessingPoint();

    /*!
    *   Move neighbor points to temporary point cloud
    */
    void moveNeighborPoints();

    /*!
    *   PCL Find Points in a radius function
    */
    void radiusSearch();

    /*!
    *   Make sure vector elements are unique
    */
    void makeUnique(std::vector<int>&, std::vector<int>&);

    /*!
    *   Searching process
    */
    void doExpandSearch();
};

#endif // EXPANDSEARCH_H
