#include "generate_viewpoint.h"

// Constructor
GenViewpoint::GenViewpoint(pcl::PointXYZ input_seed, pcl::PointNormal input_normal)
{
    seed = input_seed;
    seed_normal = input_normal;
}

void GenViewpoint::getConstraints(bool x_axis, bool y_axis, bool z_axis, double value)
{
    constraint_rule.resize(3);
    constraint_rule.at(0) = x_axis;
    constraint_rule.at(1) = y_axis;
    constraint_rule.at(2) = z_axis;
    constraint_value = value;
}

void GenViewpoint::calViewpointLoc(pcl::PointXYZ &viewpoint)
{
    double t;
    if(constraint_rule.at(0))
    {
        t = (constraint_value - seed.x) / seed_normal.normal_x; 
        viewpoint.x = constraint_value;
        viewpoint.y = seed.y + seed_normal.normal_y * t;
        viewpoint.z = seed.z + seed_normal.normal_z * t;
    }
    if(constraint_rule.at(1))
    {
        t = (constraint_value - seed.y) / seed_normal.normal_y; 
        viewpoint.x = seed.x + seed_normal.normal_x * t;
        viewpoint.y = constraint_value;
        viewpoint.z = seed.z + seed_normal.normal_z * t;
    }
    if(constraint_rule.at(2))
    {
        t = (constraint_value - seed.z) / seed_normal.normal_z; 
        viewpoint.x = seed.x + seed_normal.normal_x * t;
        viewpoint.y = seed.y + seed_normal.normal_y * t;
        viewpoint.z = constraint_value;
    }
}