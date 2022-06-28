#include "robots.h"

Robot::Robot(std::string namespace_name)
{
    namespace_name_ = namespace_name;
}

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

float UAV::get_local_offset()
{
    return local_offset_g_;
}

void UAV::set_local_offset(float offset)
{
    local_offset_g_ = offset;
}

geometry_msgs::Point UAV::get_local_offset_pose()
{
    return local_offset_pose_;
}

void UAV::set_local_offset_pose(geometry_msgs::Point local_offset_pose)
{
    local_offset_pose_ = local_offset_pose;
}

float UAV::get_local_desired_heading()
{
    return local_desired_heading_g_;
}

void UAV::set_local_desired_heading(float heading)
{
    local_desired_heading_g_ = heading;
}

float UAV::get_correction_heading()
{
    return correction_heading_g_;
}

void UAV::set_correction_heading(float correction_heading)
{
    correction_heading_g_ = correction_heading;
}

geometry_msgs::PoseStamped UAV::get_waypoint()
{
    return waypoint_;
}

void UAV::set_waypoint(geometry_msgs::PoseStamped wp)
{
    waypoint_ = wp;
}

void UAV::set_waypoint_orientation(float qw, float qx, float qy, float qz)
{
    waypoint_.pose.orientation.w = qw;
    waypoint_.pose.orientation.x = qx;
    waypoint_.pose.orientation.y = qy;
    waypoint_.pose.orientation.z = qz;
}
void UAV::set_waypoint_position(float x, float y, float z)
{
    waypoint_.pose.position.x = x;
    waypoint_.pose.position.y = y;
    waypoint_.pose.position.z = z;
}

geometry_msgs::Pose UAV::get_correction_vector()
{
    return correction_vector_;
}

std::string UAV::get_namespace_name()
{
    return namespace_name_;
}

mavros_msgs::State UAV::get_state()
{
    return current_state_;
}

void UAV::set_state(mavros_msgs::State state)
{
    current_state_ = state;
}

UAV::UAV(std::string namespace_name) : Robot(namespace_name)
{
    local_offset_g_ = 0;
    correction_heading_g_ = 0;
}

ASV::ASV(std::string namespace_name) : Robot(namespace_name)
{
}