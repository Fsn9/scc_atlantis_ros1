#ifndef ROBOTS_H
#define ROBOTS_H
#include <string>

class Robot
{
  protected:
    float battery_;
    std::string state_;
};

class UAV : public Robot
{
  protected:
    float current_heading_g_;
    float local_offset_g_;
    float correction_heading_g_;
    float local_desired_heading_g_; 
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
