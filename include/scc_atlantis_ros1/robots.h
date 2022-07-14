#ifndef ROBOTS_H
#define ROBOTS_H
#include <string>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

class Robot
{
  public:
    Robot(std::string namespace_name);
    std::string get_namespace_name();
  protected:
    float battery_;
    std::string state_;
    std::string namespace_name_;
};

class UAV : public Robot
{
  public:
    UAV(std::string namespace_name);
    nav_msgs::Odometry get_pose();
    void set_pose(nav_msgs::Odometry pose);
    float get_heading();
    void set_heading(float heading);
    float get_local_offset();
    void set_local_offset(float offset);
    geometry_msgs::Point get_local_offset_pose();
    void set_local_offset_pose(geometry_msgs::Point local_offset_pose);
    float get_local_desired_heading();
    void set_local_desired_heading(float heading);
    float get_correction_heading();
    void set_correction_heading(float correction_heading);
    geometry_msgs::PoseStamped get_waypoint();
    void set_waypoint(geometry_msgs::PoseStamped wp);
    void set_waypoint_orientation(float qw, float qx, float qy, float qz);
    void set_waypoint_position(float x, float y, float z);
    geometry_msgs::Pose get_correction_vector();
    
    mavros_msgs::State get_state();
    void set_state(mavros_msgs::State state);

  private: // TODO: erase _g
    float current_heading_g_;
    float local_offset_g_;
    float correction_heading_g_;
    float local_desired_heading_g_;

    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_pose_;
    geometry_msgs::Pose correction_vector_;
    geometry_msgs::Point local_offset_pose_;
    geometry_msgs::PoseStamped waypoint_;
};

class ASV : public Robot
{
  public:
    ASV(std::string namespace_name);
};

#endif // ROBOTS_H
