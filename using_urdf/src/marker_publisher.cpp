#include "marker_publisher.h"
#include <urdf/model.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

Marker::Marker::Marker(double inX, double inY, double inZ, double inoX, double inoY, double inoZ, double inoW, std::string inName)
    : x(inX), y(inY), z(inZ), oX(inoX), oY(inoY), oZ(inoZ), oW(inoW), name(inName)
{
}

Marker::Marker::~Marker()
{
}

void Marker::Marker::markerCallback()
{
  ros::Rate rate(10.0);
  tf::TransformListener listenerHand;
  tf::TransformListener listenerGripper_right;
  tf::TransformListener listenerGripper_left;
  while (this->n.ok())
  {
    tf::StampedTransform transformHand;
    tf::StampedTransform transformGripper_right;
    tf::StampedTransform transformGripper_left;
    try
    {
      listenerHand.lookupTransform("base_link", "hand", ros::Time(0), transformHand);
      listenerGripper_right.lookupTransform("base_link", "gripper_right", ros::Time(0), transformGripper_right);
      listenerGripper_left.lookupTransform("base_link", "gripper_left", ros::Time(0), transformGripper_left);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    if ((transformGripper_left.getOrigin().getX() < this->x + marker.scale.x / 2) &&
        (transformGripper_left.getOrigin().getX() > this->x - marker.scale.x / 2) &&
        (transformGripper_right.getOrigin().getX() < this->x + marker.scale.x / 2) &&
        (transformGripper_right.getOrigin().getX() > this->x - marker.scale.x / 2) &&

        (transformGripper_left.getOrigin().getY() < this->y + marker.scale.y / 2) &&
        (transformGripper_left.getOrigin().getY() > this->y - marker.scale.y / 2) &&
        (transformGripper_right.getOrigin().getY() < this->y + marker.scale.y / 2) &&
        (transformGripper_right.getOrigin().getY() > this->y - marker.scale.y / 2) &&

        (transformGripper_left.getOrigin().getZ() < this->z + marker.scale.z / 2) &&
        (transformGripper_left.getOrigin().getZ() > this->z - marker.scale.z / 2) &&
        (transformGripper_right.getOrigin().getZ() < this->z + marker.scale.z / 2) &&
        (transformGripper_right.getOrigin().getZ() > this->z - marker.scale.z / 2))
    {
      if (this->stucked == false)
      {
        differenceX = std::abs(((transformGripper_right.getOrigin().getX() + transformGripper_left.getOrigin().getX()) / 2) - this->x);
        differenceY = std::abs(((transformGripper_right.getOrigin().getY() + transformGripper_left.getOrigin().getY()) / 2) - this->y);
        differenceZ = std::abs(((transformGripper_right.getOrigin().getZ() + transformGripper_left.getOrigin().getZ()) / 2) - this->z);

        differenceoX = std::abs(transformHand.getRotation().getX() - this->oX);
        differenceoY = std::abs(transformHand.getRotation().getY() - this->oY);
        differenceoZ = std::abs(transformHand.getRotation().getZ() - this->oZ);
        differenceoW = std::abs(transformHand.getRotation().getW() - this->oW);
      }
      this->stucked = true;
      this->x = ((transformGripper_right.getOrigin().getX() + transformGripper_left.getOrigin().getX()) / 2) + differenceX;
      this->y = ((transformGripper_right.getOrigin().getY() + transformGripper_left.getOrigin().getY()) / 2) + differenceY;
      this->z = ((transformGripper_right.getOrigin().getZ() + transformGripper_left.getOrigin().getZ()) / 2) + differenceZ;
      // this->oX = (transformHand.getRotation().getX() - differenceoX) ;
      //this->oY = (transformHand.getRotation().getY() - differenceoY) ;
      //this->oZ = (transformHand.getRotation().getZ() - differenceoZ) ;
      //this->oW = (transformHand.getRotation().getW() - differenceoW) ;


      this->displayMarker();
    }
    else
    {
      this->stucked = false;

    
    }
    if (this->stucked == false && (this->z > 0))
    {
 
      this->z =this->z- 0.000005;
    
      this->displayMarker();
    }
  }
}

void Marker::Marker::displayMarker()
{
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = this->name;
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = this->x;
  marker.pose.position.y = this->y;
  marker.pose.position.z = this->z;
  marker.pose.orientation.x = this->oX;
  marker.pose.orientation.y = this->oY;
  marker.pose.orientation.z = this->oZ;
  marker.pose.orientation.w = this->oW;

  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker");
  std::cout << "MARKER" << std::endl;
  Marker::Marker m(0.19, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, "cyllinder1");
  sleep(5);
  m.displayMarker();
  m.markerCallback();

  ros::spin();
  // Set our initial shape type to be a cube
}
