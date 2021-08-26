#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>
#include <stdlib.h>

#include <pcl/visualization/pcl_visualizer.h>

/*!
*   \ingroup    ac_clustering Clustering
*   \brief      Clustering class
*   \details    This class perform clustering processes.\n
*/
class Clustering
{
public:
    /*!
    *   Clustering class constructor
    */
    Clustering();

protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_; // Pointer for input cloud

    /*!
    *   Get input non-colored cloud
    */
    void getInputNonColored(pcl::PointCloud<pcl::PointXYZ>::Ptr);

    /*!
    *   Get input colored cloud
    */
    void getInputColored(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
};

#endif // EXPANDSEARCH_H
