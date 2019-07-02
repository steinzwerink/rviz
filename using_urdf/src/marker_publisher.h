#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "tf2_msgs/TFMessage.h"
#include "std_msgs/String.h"
#include <string>
#include <thread>

namespace Marker
{
class Marker
{
public:
    Marker(double inX, double inY, double inZ, double inoX, double inoY, double inoZ, double inoW, std::string inName);
    virtual ~Marker();
    void markerCallback();
    ros::NodeHandle n;
    static void displayMarker(Marker* m);

private:
    double x;
    double y;
    double z;
    double oX;
    double oY;
    double oZ;
    double oW;
    double differenceX = 0;
    double differenceY = 0;
    double differenceZ = 0;

    double differenceoX = 0;
    double differenceoY = 0;
    double differenceoZ = 0;
    double differenceoW = 0;
    bool stucked = false;
    bool checkStucked(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left);
    void setDifferences(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left);
    bool isGripperLeftCollision(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left);
    bool isGripperRightCollision(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left);
    std::string name;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    visualization_msgs::Marker marker;
};
} // namespace Marker