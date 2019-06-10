#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include "using_urdf/pose.h"

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <std_msgs/String.h>

namespace publisher
{
class Publisher
{
private:
public:
    Publisher();
    virtual ~Publisher();
    std::string convertMessage(pose::pose p);
};

} // namespace publisher
#endif