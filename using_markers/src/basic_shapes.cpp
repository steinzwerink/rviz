#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->position[0]);
}

int main( int argc, char** argv )
{
    
  
}