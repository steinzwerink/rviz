#include <ros/ros.h>
#include <string>
#include "using_urdf/pose.h"

pose::pose::pose()
{
}

pose::pose::pose(const std::string &pose_name, const int &duration)
    : pose_name(pose_name), duration(duration)
{
}

pose::pose::~pose()
{
}

pose::pose &pose::pose::operator=(const pose &other)
{
    if (this != &other)
    {
        pose_name = other.pose_name;
    }
    return *this;
}