#include "fov_check.h"

// Constructor
FOVCheck::FOVCheck(pcl::PointCloud<pcl::PointXYZ> cloud, double seed_index, pcl::PointXYZ viewpoint)
{
    input_cloud = cloud;
    sensor_pos = viewpoint;
    seed_idx = seed_index;

    fov_tag.resize(input_cloud.size());
    visited_tag.resize(input_cloud.size());
    for(int i=0; i<input_cloud.size(); i++)
    {
        fov_tag.at(i) = false;
        visited_tag.at(i) = false;
    }

    finished = false;
}

void FOVCheck::reset()
{
    input_cloud.clear();
    normal.clear();
    weighting.clear();
    fov_tag.clear();
    visited_tag.clear();
    fov_neighbours_idx.clear();
}

void FOVCheck::initiateFOVCheck(std::vector<int> &fov_set_idx)
{
    // First loop, look for immediate neighbour of the seed
    std::vector<int> new_neighbours_idx;
    fov_tag.at(seed_idx) = true;
    visited_tag.at(seed_idx) = true;
    this->getImmediateNeighbours(input_cloud.at(seed_idx), new_neighbours_idx);
    std::cout << "First neighbour set size: " << new_neighbours_idx.size() << std::endl;
    this->checkFOVCover(new_neighbours_idx);
    this->removeNotFOVNeighbours(new_neighbours_idx);
    // this->addToFOV();
    std::cout << "FOV size: " << new_neighbours_idx.size() << std::endl;
    std::cout << "Immediate neighbour size: " << new_neighbours_idx.size() << std::endl;
    int count_fov_tag = 0;
    int count_visted_tag = 0;
    for(int i=0; i<fov_tag.size(); i++)
    {
        if(fov_tag.at(i)) count_fov_tag++;
        if(visited_tag.at(i)) count_visted_tag++;
    }
    std::cout << "Tags size: fov " << count_fov_tag << " visited " << count_visted_tag << std::endl; 

    // Start looping till no new points can be added
    while(!finished)
    {
        new_neighbours_idx.clear();
        for(int i=0; i<fov_neighbours_idx.size(); i++)
        {
            std::vector<int> temp_neighbour_idx;
            this->getImmediateNeighbours(input_cloud.at(fov_neighbours_idx.at(i)), temp_neighbour_idx);
            this->updateNeighbours(temp_neighbour_idx, new_neighbours_idx);
        }

        std::cout << "Immediate neighbour size, visited removed: " << new_neighbours_idx.size() << std::endl;
        this->removeDup(new_neighbours_idx);
        std::cout << "Immediate neighbour size, dup removed: " << new_neighbours_idx.size() << std::endl;
        this->checkFOVCover(new_neighbours_idx);
        this->removeNotFOVNeighbours(new_neighbours_idx);
        // this->addToFOV();
        std::cout << "FOV size: " << fov_neighbours_idx.size() << std::endl;
        std::cout << "Total neighbour size: " << new_neighbours_idx.size() << std::endl;
        // int count_fov_tag = 0;
        // int count_visted_tag = 0;
        // for (int i = 0; i < fov_tag.size(); i++)
        // {
        //     if (fov_tag.at(i))
        //         count_fov_tag++;
        //     if (visited_tag.at(i))
        //         count_visted_tag++;
        // }
        // std::cout << "Tags size: fov " << count_fov_tag << " visited " << count_visted_tag << std::endl;
    }    

    fov_set_idx = fov_neighbours_idx;
}

void FOVCheck::getImmediateNeighbours(pcl::PointXYZ &point, std::vector<int> &neighbour_idx)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_ptr = input_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_ptr);
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(point, IMMEDIATE_NEIGHBOURS_RANGE, neighbour_idx, pointRadiusSquaredDistance);
}

void FOVCheck::checkFOVCover(std::vector<int> &neighbour_idx)
{
    finished = true;
    for(int i=0; i<neighbour_idx.size(); i++)
    {
        double angle;
        calAngle(input_cloud.at(seed_idx), sensor_pos,input_cloud.at(neighbour_idx.at(i)),angle);
        // If is inside FOV
        if(fabs(angle) < FOV_LIMITS)
        {
            fov_tag.at(neighbour_idx.at(i)) = true;
            finished = false;
        }
        // If not in FOV
        else 
        {
            fov_tag.at(neighbour_idx.at(i)) = false;
        }

        visited_tag.at(neighbour_idx.at(i)) = true;
    }
}

void FOVCheck::updateNeighbours(std::vector<int> &temp_neighbour_idx, std::vector<int> &nonvisited_neighbour_idx)
{
    for(int i=0; i<temp_neighbour_idx.size(); i++)
    {
        if(visited_tag.at(temp_neighbour_idx.at(i)) == false)
            nonvisited_neighbour_idx.push_back(temp_neighbour_idx.at(i));
    }
}

void FOVCheck::removeNotFOVNeighbours(std::vector<int> &nonvisited_neighbour_idx)
{   
    // fov_neighbours_idx.clear();
    for(int i=0; i<nonvisited_neighbour_idx.size(); i++)
    {
        if(fov_tag.at(nonvisited_neighbour_idx.at(i)) == true)
            fov_neighbours_idx.push_back(nonvisited_neighbour_idx.at(i));
    }
}

void FOVCheck::removeDup(std::vector<int> &vec)
{
    auto end = vec.end();
    for (auto it = vec.begin(); it != end; ++it) {
        end = std::remove(it + 1, end, *it);
    }
 
    vec.erase(end, vec.end());
}

void FOVCheck::calAngle(pcl::PointXYZ &seed, pcl::PointXYZ &viewpoint, pcl::PointXYZ &current_point, double &result)
{
    Eigen::Vector3f viewpoint_to_seed, viewpoint_to_point;
    
    viewpoint_to_seed(0) = - viewpoint.x + seed.x;
    viewpoint_to_seed(1) = - viewpoint.y + seed.y;
    viewpoint_to_seed(2) = - viewpoint.z + seed.z;

    viewpoint_to_point(0) = - viewpoint.x + current_point.x;
    viewpoint_to_point(1) = - viewpoint.y + current_point.y;
    viewpoint_to_point(2) = - viewpoint.z + current_point.z;

    result = std::acos(viewpoint_to_point.dot(viewpoint_to_seed) / (viewpoint_to_point.norm() * viewpoint_to_seed.norm()));
}