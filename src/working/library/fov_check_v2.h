#ifndef FOVCHECKV2_H
#define FOVCHECKV2_H

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <vector>
#include <stdlib.h>
#include <algorithm>

/*!
*   \details    This class look for thr points in the FOV of a sensor
*/
class FOVCheckV2
{
public:
    const double INITIAL_RANGE = 1.8;
    const double RANGE_INCREASE_STEP = 2;
    const double FOV_LIMITS = 20 * M_PI / 180;
    const float VOXEL_SIZE = 0.5;

    /*!
    *   FoVCheckV2 class constructor
    */
    FOVCheckV2(pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointNormal>&, int&, pcl::PointNormal&);

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
    *   Find all points in radius
    */
    void radiusSearch(double, std::vector<int>&);

    /*!
    *   Check the angle of all points in range
    */
    void checkAngle(std::vector<int>&, std::vector<int>&);

    /*!
    *   Remove all old points, keep new points from search function
    */
    void findNotPreviouslyExistedPoints(std::vector<int>&, std::vector<int>&);

    /*!
    *   Calculate the angle of a point to the viewpoint
    */
    void calAngle(pcl::PointXYZ&, double&);

    /*!
    *   Check a point angle, for comparison with viewpoint normal
    */
    void pointNormalCheck(int&, double&);

    /*!
    *   Check for visibility (Line of sight)
    */
    void visCheck(int&, std::vector<int>&, bool&);

    // Inputs
    pcl::PointCloud<pcl::PointXYZ> _input_cloud;
    pcl::PointCloud<pcl::PointNormal> _input_cloud_normal;
    pcl::PointXYZ _fov_center;
    int _fov_idx;
    pcl::PointNormal _viewpoint;
    bool _break_expand_flag;
};

#endif // FOVCHECKV2_H
