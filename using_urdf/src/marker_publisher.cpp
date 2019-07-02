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
      //ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    if (checkStucked(transformGripper_left, transformGripper_right))
    {
      if (this->stucked == false)
      {
        setDifferences(transformGripper_left, transformGripper_right);
      }
      this->stucked = true;
      this->x = ((transformGripper_right.getOrigin().getX() + transformGripper_left.getOrigin().getX()) / 2) + differenceX;
      this->y = ((transformGripper_right.getOrigin().getY() + transformGripper_left.getOrigin().getY()) / 2) + differenceY;
      this->z = ((transformGripper_right.getOrigin().getZ() + transformGripper_left.getOrigin().getZ()) / 2) + differenceZ;
      // this->oX = (transformHand.getRotation().getX() - differenceoX) ;
      //this->oY = (transformHand.getRotation().getY() - differenceoY) ;
      //this->oZ = (transformHand.getRotation().getZ() - differenceoZ) ;
      //this->oW = (transformHand.getRotation().getW() - differenceoW) ;

      std::thread t1(this->displayMarker, this);
      t1.join();
      // this->displayMarker();
    }
    else
    {
      this->stucked = false;
    }

    if (this->stucked == false && this->z < 0.3)
    {
      if (isGripperLeftCollision(transformGripper_left, transformGripper_right))
      {
        this->x = transformGripper_left.getOrigin().getX() + differenceX;
        this->y = transformGripper_left.getOrigin().getY() + differenceY;
        this->z = transformGripper_left.getOrigin().getZ() + differenceZ;
      }
      if (isGripperRightCollision(transformGripper_left, transformGripper_right))
      {
        this->x = transformGripper_right.getOrigin().getX() + differenceX;
        this->y = transformGripper_right.getOrigin().getY() + differenceY;
        this->z = transformGripper_right.getOrigin().getZ() + differenceZ;
      }
    }
    if (this->stucked == false && (this->z > 0))
    {

      this->z = this->z - 0.000005;
      //this->displayMarker();
      std::thread t2(this->displayMarker, this);
      t2.join();
    }
  }
}

void Marker::Marker::displayMarker(Marker *m)
{
  m->marker.header.frame_id = "/base_link";
  m->marker.header.stamp = ros::Time::now();
  m->marker.ns = m->name;
  m->marker.id = 0;
  m->marker.type = m->shape;
  m->marker.action = visualization_msgs::Marker::ADD;
  m->marker.pose.position.x = m->x;
  m->marker.pose.position.y = m->y;
  m->marker.pose.position.z = m->z;
  m->marker.pose.orientation.x = m->oX;
  m->marker.pose.orientation.y = m->oY;
  m->marker.pose.orientation.z = m->oZ;
  m->marker.pose.orientation.w = m->oW;

  m->marker.scale.x = 0.03;
  m->marker.scale.y = 0.03;
  m->marker.scale.z = 0.03;

  m->marker.color.r = 1.0f;
  m->marker.color.g = 0.0f;
  m->marker.color.b = 0.0f;
  m->marker.color.a = 1.0;
  m->marker.lifetime = ros::Duration();
  m->marker_pub.publish(m->marker);
}

bool Marker::Marker::checkStucked(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left)
{
  return ((transformGripper_left.getOrigin().getX() < this->x + marker.scale.x / 2) &&
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
          (transformGripper_right.getOrigin().getZ() > this->z - marker.scale.z / 2));
}

bool Marker::Marker::isGripperLeftCollision(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left)
{
  return ((transformGripper_left.getOrigin().getX() < this->x + marker.scale.x / 2) &&
          (transformGripper_left.getOrigin().getX() > this->x - marker.scale.x / 2) &&
          (transformGripper_left.getOrigin().getY() < this->y + marker.scale.y / 2) &&
          (transformGripper_left.getOrigin().getY() > this->y - marker.scale.y / 2) &&
          (transformGripper_left.getOrigin().getZ() < this->z + marker.scale.z / 2) &&
          (transformGripper_left.getOrigin().getZ() > this->z - marker.scale.z / 2));
}

bool Marker::Marker::isGripperRightCollision(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left)
{
  return ((transformGripper_right.getOrigin().getX() < this->x + marker.scale.x / 2) &&
          (transformGripper_right.getOrigin().getX() > this->x - marker.scale.x / 2) &&
          (transformGripper_right.getOrigin().getY() < this->y + marker.scale.y / 2) &&
          (transformGripper_right.getOrigin().getY() > this->y - marker.scale.y / 2) &&
          (transformGripper_right.getOrigin().getZ() < this->z + marker.scale.z / 2) &&
          (transformGripper_right.getOrigin().getZ() > this->z - marker.scale.z / 2));
}

void Marker::Marker::setDifferences(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left)
{
  differenceX = std::abs(((transformGripper_right.getOrigin().getX() + transformGripper_left.getOrigin().getX()) / 2) - this->x);
  differenceY = std::abs(((transformGripper_right.getOrigin().getY() + transformGripper_left.getOrigin().getY()) / 2) - this->y);
  differenceZ = std::abs(((transformGripper_right.getOrigin().getZ() + transformGripper_left.getOrigin().getZ()) / 2) - this->z);
  // differenceoX = std::abs(transformHand.getRotation().getX() - this->oX);
  // differenceoY = std::abs(transformHand.getRotation().getY() - this->oY);
  // differenceoZ = std::abs(transformHand.getRotation().getZ() - this->oZ);
  // differenceoW = std::abs(transformHand.getRotation().getW() - this->oW);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker");
  std::cout << "MARKER" << std::endl;
  Marker::Marker m(0.19, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, "cyllinder1");
  sleep(5);
  //m.displayMarker();
  std::thread t1(m.displayMarker, &m);
  t1.join();

  m.markerCallback();

  ros::spin();
  // Set our initial shape type to be a cube
}
