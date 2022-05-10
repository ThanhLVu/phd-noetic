#ifndef GENVIEWPOINT_H
#define GENVIEWPOINT_H

#include "pcl/point_types.h"

#include "Eigen/Dense"
#include <vector>
#include <stdlib.h>
#include <algorithm>

/*!
*   \details    This class generate the viewpoints with different angle
*/
class GenViewpoint
{
public:
    /*!
    *   GenViewpoint class constructor
    */
    GenViewpoint(pcl::PointXYZ, pcl::PointNormal);

    /*!
    *   Get some constraints of the viewpoints
    */
    void getConstraints(bool, bool, bool, double);

    /*!
    *   Calculate viewpoint location
    */
    void calViewpointLoc(pcl::PointXYZ&);

protected:

    std::vector<bool> constraint_rule;
    double constraint_value;
    pcl::PointXYZ seed;
    pcl::PointNormal seed_normal;
    
};

#endif // GENVIEWPOINT
