#include "expand_search.h"

// Constructor
ExpandSearch::ExpandSearch(pcl::PointCloud<pcl::PointXYZ> input_cloud, double voxel_grid_size)
: input_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  processing_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  temp_neighbor_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  clustered_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
  kd_tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>)
//  indices_extract_(new pcl::ExtractIndices<pcl::PointXYZ>)
{
    new_cluster_ = true;
    neighbor_distance_vector_.clear();
    neighbor_indices_vector_.clear();
    this->getInputCloud(input_cloud);
    this->getVoxelGridSize(voxel_grid_size);
    this->doExpandSearch();
}

// Get input cloud function
void ExpandSearch::getInputCloud(pcl::PointCloud<pcl::PointXYZ> input_cloud)
{
    *input_cloud_ = input_cloud;
    *processing_cloud_ = input_cloud;
}

// Get input cloud voxel grid size function
void ExpandSearch::getVoxelGridSize(double voxel_grid_size)
{
    searching_radius_ = 2 * voxel_grid_size;
}

// Generate random first point index
int ExpandSearch::generateRandomNumber()
{
    // Initialize random seed:
    srand (time(NULL));

    return rand();
}

// Randomize color code of current cluster
uint32_t ExpandSearch::randomizeColorCode()
{
    uint8_t r = this->generateRandomNumber() % 255;
    uint8_t g = this->generateRandomNumber() % 255;
    uint8_t b = this->generateRandomNumber() % 255;
    current_cluster_color_ = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
}

// Move processing point to clustered point cloud
void ExpandSearch::moveProcessingPoint() {
    pcl::PointXYZRGB processing_point;
    processing_point.x = processing_cloud_->at(processing_point_index_).x;
    processing_point.y = processing_cloud_->at(processing_point_index_).y;
    processing_point.y = processing_cloud_->at(processing_point_index_).z;

    if (new_cluster_)
        this->randomizeColorCode();
    processing_point.rgb = current_cluster_color_;

    processing_cloud_->erase(processing_cloud_->begin() + processing_point_index_);
    clustered_cloud_->push_back(processing_point);
}

// Move neighbor points to temporary set
void ExpandSearch::moveNeighborPoints()
{
    std::cout << "Inside moveNeighborPoints: processing_cloud: " << processing_cloud_->points.size() << " neighbor_indices_vector: " << neighbor_indices_vector_unique_.size() << std::endl;
    pcl::PointIndices::Ptr neighbor_indices(new pcl::PointIndices);
    neighbor_indices->indices = neighbor_indices_vector_unique_;

    pcl::ExtractIndices<pcl::PointXYZ>::Ptr indices_extract_(new pcl::ExtractIndices<pcl::PointXYZ>);
    indices_extract_->setInputCloud(processing_cloud_);
    indices_extract_->setIndices(neighbor_indices);
    indices_extract_->filter(*temp_neighbor_cloud_);
    indices_extract_->setNegative(true);
//    indices_extract_->setKeepOrganized (true);
    indices_extract_->filter(*processing_cloud_);

    neighbor_indices_vector_.clear();
    neighbor_distance_vector_.clear();
    neighbor_indices_vector_unique_.clear();

    std::cout << "Inside moveNeighborPoints: processing_cloud: " << processing_cloud_->points.size() << " neighbor_indices_vector: " << neighbor_indices_vector_unique_.size() << std::endl;
    std::cout << "Inside moveNeighborPoints: temp_neighbor_cloud: " << temp_neighbor_cloud_->points.size() << std::endl;

}

// Do radius search for neighbors
void ExpandSearch::radiusSearch()
{
    std::vector<int> temp_indices_vector;
    std::vector<float> temp_distance_vector;

    // Generate random point index
    kd_tree_->setInputCloud(processing_cloud_);
    kd_tree_->radiusSearch(processing_point_index_,
                           searching_radius_,
                           temp_indices_vector,
                           temp_distance_vector);

    this->moveProcessingPoint();

    for(int i=0; i<temp_indices_vector.size(); i++)
    {
        neighbor_indices_vector_.push_back(temp_indices_vector.at(i));
        neighbor_distance_vector_.push_back(temp_distance_vector.at(i));
    }
}

// Make sure vector elements are unique
void ExpandSearch::makeUnique(std::vector<int>& input, std::vector<int>& output)
{
    output = input;
    bool unique = false;

    while(!unique)
    {
        unique = true;
        for(int i=0; i<output.size()-1; i++)
        {
            for(int j = i+1; j<output.size(); j++)
            {
                if(output.at(i) == output.at(j))
                {
                    output.erase(output.begin() + i);
                    unique = false;
                }
            }
        }
    }
}

void ExpandSearch::doExpandSearch()
{
    int cnt = 0;
    while (processing_cloud_->points.size() > 0)
    {
        if(new_cluster_)
        {
            processing_point_index_ = this->generateRandomNumber() % input_cloud_->points.size();
            new_cluster_ = false;

            this->radiusSearch();
            this->makeUnique(neighbor_indices_vector_, neighbor_indices_vector_unique_);
            std::cout << "I did this only once, right?" << std::endl;
        }
        else
        {
            if(neighbor_indices_vector_unique_.size() > 1)
            {
                this->moveNeighborPoints();

                for(int i=0; i<temp_neighbor_cloud_->points.size(); i++)
                {
                    processing_cloud_->push_back(temp_neighbor_cloud_->at(i));
                    processing_point_index_ = processing_cloud_->size()-1;
                    this->radiusSearch();
                }

                this->makeUnique(neighbor_indices_vector_, neighbor_indices_vector_unique_);
            }
            else
            {
                std::cout << "===========> End of cluster, Start new one " << std::endl;
                std::cout << "Clustered cloud size: " << clustered_cloud_->points.size() << std::endl;

                pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
                viewer.showCloud (clustered_cloud_);
                while (!viewer.wasStopped ())
                {
                }
            }
        }
    }
}

