#ifndef FOVCHECK_H
#define FOVCHECK_H

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <vector>
#include <stdlib.h>
#include <algorithm>

/*!
*   \details    This class look for thr points in the FOV of a sensor
*/
class FOVCheck
{
public:
    const double IMMEDIATE_NEIGHBOURS_RANGE = 1.8;
    const double FOV_LIMITS = 12 * M_PI / 180;
    /*!
    *   FoVCheck class constructor
    */
    FOVCheck(pcl::PointCloud<pcl::PointXYZ>, double, pcl::PointXYZ);

    /*!
    *   Reset the fov data
    */
    void reset();

    /*!
    *   Reset the fov data
    */
    void initiateFOVCheck(std::vector<int>&); 

protected:
    /*!
    *   Get the immediate neighbour of a point
    */
    void getImmediateNeighbours(pcl::PointXYZ&, std::vector<int>&);

    /*!
    *   Check if neighbours are in FOV
    */
    void checkFOVCover(std::vector<int>&);

    /*!
    *   Update the set of new neighbour index
    */
    void updateNeighbours(std::vector<int>&, std::vector<int>&);

    /*!
    *   Remove the neighbours that are not inside FOV
    */
    void removeNotFOVNeighbours(std::vector<int>&);

    /*!
    *   Remove duplilcate values from vector
    */
    void removeDup(std::vector<int>&);

    /*!
    *   Calculate the angle of a point to the viewpoint
    */
    void calAngle(pcl::PointXYZ&, pcl::PointXYZ&, pcl::PointXYZ&, double&);

    // Inputs
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::PointCloud<pcl::PointNormal> normal;
    std::vector<double> weighting;
    int seed_idx;
    
    // Tags
    std::vector<bool> fov_tag;
    std::vector<bool> visited_tag;

    // For this class
    std::vector<int> fov_region_idx;
    bool finished;
    std::vector<int> fov_neighbours_idx;
    pcl::PointXYZ sensor_pos;
};

#endif // FOVCHECK_H
