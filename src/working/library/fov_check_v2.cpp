#include "fov_check_v2.h"

// Constructor
FOVCheckV2::FOVCheckV2(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointNormal> &normals, int &fov_center_idx, pcl::PointNormal &viewpoint)
{
    _input_cloud = cloud;
    _input_cloud_normal = normals;
    _fov_center_idx = fov_center_idx;
    _fov_center = _input_cloud.at(_fov_center_idx);
    _viewpoint = viewpoint;
    _break_expand_flag = false;
}

void FOVCheckV2::reset()
{    
}

void FOVCheckV2::initiateFOVCheck(std::vector<int> &fov_set_idx)
{
    // First expansion
    octomap::OcTree oc_tree(1);
    oc_tree.clear();
    // oc_tree.updateNode(_viewpoint.x, _viewpoint.y, _viewpoint.z, false);

    std::vector<int> old_points_in_range_idx;
    std::vector<int> new_points_in_range_idx;
    std::vector<int> points_in_fov_idx;
    this->radiusSearch(INITIAL_RANGE, new_points_in_range_idx);
    std::cout << "First loop radius search, points in range: " << new_points_in_range_idx.size() << std::endl;
    this->checkAngle(new_points_in_range_idx, points_in_fov_idx, oc_tree);
    std::cout << "First loop radius search, points in fov: " << points_in_fov_idx.size() << std::endl;
    this->updateOctree(new_points_in_range_idx, oc_tree);
    old_points_in_range_idx = new_points_in_range_idx;
    
    int counter = 1;
    // After 1st expansion
    while(!_break_expand_flag)
    {
        this->radiusSearch(INITIAL_RANGE + counter * RANGE_INCREASE_STEP, new_points_in_range_idx);
        std::cout << counter << " loop radius search, points in range: " << new_points_in_range_idx.size() << std::endl;
        this->findNotPreviouslyExistedPoints(old_points_in_range_idx, new_points_in_range_idx);
        std::cout << counter << " loop radius search, new points found: " << new_points_in_range_idx.size() << std::endl;
        this->checkAngle(new_points_in_range_idx, points_in_fov_idx, oc_tree);
        std::cout << counter << " loop radius search, points in fov: " << points_in_fov_idx.size() << std::endl;
        this->updateOctree(new_points_in_range_idx, oc_tree);
        counter++;
        // if(counter == 2) break;
    }
    fov_set_idx = points_in_fov_idx;
}

void FOVCheckV2::checkAngle(std::vector<int> &points_in_range_idx, std::vector<int> &points_in_fov_idx, octomap::OcTree& oc_tree)
{
    _break_expand_flag = true;
    std::vector<int> old_points_in_fov_idx;
    old_points_in_fov_idx = points_in_fov_idx;
    for(int i=0; i<points_in_range_idx.size(); i++)
    {
        double angle;
        calAngle(_input_cloud.at(points_in_range_idx.at(i)),angle);
        // double angle_normal_check;
        // pointNormalCheck(points_in_range_idx.at(i), angle_normal_check);
        // If is inside FOV
        // if(fabs(angle) < FOV_LIMITS && angle_normal_check > 0.5) 
        bool visible = false;
        visCheck(points_in_range_idx.at(i), oc_tree, visible);
        if(fabs(angle) < FOV_LIMITS && visible)
        // if(fabs(angle) < FOV_LIMITS) 
        {
            points_in_fov_idx.push_back(points_in_range_idx.at(i));
            _break_expand_flag = false;
        }
    }
}

void FOVCheckV2::findNotPreviouslyExistedPoints(std::vector<int> &old_points_in_range, std::vector<int> &new_points_in_range)
{
    std::vector<int> temp_storage;
    temp_storage = new_points_in_range;
    for(int i=0; i< new_points_in_range.size(); i++)
    {
        auto result = std::find(begin(old_points_in_range), end(old_points_in_range), new_points_in_range.at(i));
        if(result != std::end(old_points_in_range))
        {
            new_points_in_range.erase(new_points_in_range.begin() + i);
            i--;   
        } 
    }
    old_points_in_range = temp_storage;
}

void FOVCheckV2::radiusSearch(double range, std::vector<int> &points_in_range_idx)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_ptr = _input_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_ptr);
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(_fov_center, range, points_in_range_idx, pointRadiusSquaredDistance);
}

void FOVCheckV2::calAngle(pcl::PointXYZ &current_point, double &result)
{
    Eigen::Vector3f viewpoint_to_fov_center, viewpoint_to_current_point;
    
    viewpoint_to_fov_center(0) = - _viewpoint.x + _fov_center.x;
    viewpoint_to_fov_center(1) = - _viewpoint.y + _fov_center.y;
    viewpoint_to_fov_center(2) = - _viewpoint.z + _fov_center.z;

    viewpoint_to_current_point(0) = - _viewpoint.x + current_point.x;
    viewpoint_to_current_point(1) = - _viewpoint.y + current_point.y;
    viewpoint_to_current_point(2) = - _viewpoint.z + current_point.z;

    result = std::acos(viewpoint_to_current_point.dot(viewpoint_to_fov_center) / (viewpoint_to_current_point.norm() * viewpoint_to_fov_center.norm()));
}

void FOVCheckV2::pointNormalCheck(int &current_point_idx, double &result)
{
    Eigen::Vector3f viewpoint_to_fov_center;
    
    viewpoint_to_fov_center(0) = - _viewpoint.x + _fov_center.x;
    viewpoint_to_fov_center(1) = - _viewpoint.y + _fov_center.y;
    viewpoint_to_fov_center(2) = - _viewpoint.z + _fov_center.z;

    Eigen::Vector3f current_point_normal(_input_cloud_normal.at(current_point_idx).normal_x,
                                         _input_cloud_normal.at(current_point_idx).normal_y,
                                         _input_cloud_normal.at(current_point_idx).normal_z);

    result = current_point_normal.dot(viewpoint_to_fov_center) / (current_point_normal.norm() * viewpoint_to_fov_center.norm());
}

void FOVCheckV2::visCheck(int &current_point_idx, octomap::OcTree &oc_tree, bool &result)
{
    octomap::point3d origin(_viewpoint.x, _viewpoint.y, _viewpoint.z);
    octomap::point3d direction(_input_cloud.at(current_point_idx).x - _viewpoint.x,
                               _input_cloud.at(current_point_idx).y - _viewpoint.y,
                               _input_cloud.at(current_point_idx).z - _viewpoint.z);
    octomap::point3d end;

    bool hit_something = oc_tree.castRay(origin, direction, end, true, 200);

    if(hit_something == false) result = true;
    else
        result = false;
}

void FOVCheckV2::updateOctree(std::vector<int> &new_points_in_range_idx, octomap::OcTree &oc_tree)
{
    for(int i=0; i< new_points_in_range_idx.size(); i++)
    {
        octomap::point3d new_point(_input_cloud.at(new_points_in_range_idx.at(i)).x,
                                   _input_cloud.at(new_points_in_range_idx.at(i)).y,
                                   _input_cloud.at(new_points_in_range_idx.at(i)).z);
        oc_tree.updateNode(new_point, true);
    }
}