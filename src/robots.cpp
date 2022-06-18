#include "robots.h"

nav_msgs::Odometry UAV::get_pose()
{
    return current_pose_;
}

void UAV::set_pose(nav_msgs::Odometry pose)
{
    current_pose_ = pose;
}

float UAV::get_heading()
{
    return current_heading_g_;
}

void UAV::set_heading(float heading)
{
    current_heading_g_ = heading;
}

Zarco::Zarco()
{

}

Crow::Crow()
{
    correction_heading_g_ = 0;
}

Raven::Raven()
{
    correction_heading_g_ = 0;
}