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
  protected:
    float battery_;
    std::string state_;
};

class UAV : public Robot
{
  public:
    nav_msgs::Odometry get_pose();
    void set_pose(nav_msgs::Odometry pose);
    float get_heading();
    void set_heading(float heading);
  protected:
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

class Zarco : public Robot
{
  public:
    Zarco();
};

class Crow : public UAV
{
	public:
    Crow();
};

class Raven : public UAV
{
	public:
    Raven();
};
#endif // ROBOTS_H
