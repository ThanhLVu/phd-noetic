#include "clustering.h"

// Constructor
Clustering::Clustering()
: input_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{

}

// Get input non-colored cloud 
void Clustering::getInputNonColored(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{

}

// Get input colored cloud 
void Clustering::getInputColored(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

}