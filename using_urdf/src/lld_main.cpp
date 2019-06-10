#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <thread>
#include <string>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>
#include <using_urdf/communicatorAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servoServer");

    //communicatorAction servo("AL5D");

    ros::spin();

    return 0;
}
