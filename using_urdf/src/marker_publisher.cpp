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

void Marker::Marker::checkDirections(tf::StampedTransform transformGripper_right, tf::StampedTransform transformGripper_left)
{

  if (transformGripper_right.getOrigin().getZ() != lastRightZ)
  {
    float a = transformGripper_right.getOrigin().getZ();
    float b = lastRightZ;

    float c = transformGripper_right.getOrigin().getY();
    float d = lastRightY;

    // ROS_ERROR("%lf", transformGripper_right.getOrigin().getX());

    if (a == b)
    {
      controlNumbers.push_back(a);
      controlNumbers.push_back(b);

      if (controlNumberscounter == 2)
      {
        controlNumberscounter = 0;

        if (std::adjacent_find(controlNumbers.begin(), controlNumbers.end(), std::not_equal_to<>()) == controlNumbers.end())
        {
          if (transformGripper_right.getOrigin().getX() > 0)
          {
            if (lastRightYvals.size() % 10 == 0)
            {
              if (transformGripper_right.getOrigin().getY() < lastRightYvals[0])
              {
                this->grippers[0].movingDirectionD = movingDirection::right;
                ROS_ERROR("RIGHT");
              }
              if (transformGripper_right.getOrigin().getY() > lastRightYvals[0])
              {
                this->grippers[0].movingDirectionD = movingDirection::left;
                ROS_ERROR("LEFT");
              }
              lastRightYvals.clear();
            }
          }
        }
        else
        {
          this->grippers[0].movingDirectionD = movingDirection::notMoving;
        }

        controlNumbers.clear();
      }
      ++controlNumberscounter;
    }
  }
}

void Marker::Marker::markerCallback()
{
  //ros::Rate rate(10.0);
  tf::TransformListener listenerHand;
  tf::TransformListener listenerGripper_right;
  tf::TransformListener listenerGripper_left;
  int counter = 0;
  int lCounter = 0;
  double differenceLeft = 0;
  double differenceRight = 0;
  double differenceRightTest = 0;

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
    checkDirections(transformGripper_right, transformGripper_left);

    if (isGripperLeftCollision(transformGripper_left, transformGripper_right) && isGripperRightCollision(transformGripper_left, transformGripper_right))
    {
      if (this->stucked == false)
      {
        setDifferences(transformGripper_left, transformGripper_right);
      }
      this->stucked = true;
      this->x = ((transformGripper_right.getOrigin().getX() + transformGripper_left.getOrigin().getX()) / 2) + differenceX;
      this->y = ((transformGripper_right.getOrigin().getY() + transformGripper_left.getOrigin().getY()) / 2) + differenceY;
      this->z = ((transformGripper_right.getOrigin().getZ() + transformGripper_left.getOrigin().getZ()) / 2) + differenceZ;
    }
    else
    {
      this->stucked = false;
    }

    if (this->stucked == false && this->z < 0.1 && this->dropped == true)
    {
      if (isGripperLeftCollision(transformGripper_left, transformGripper_right)) //checks right collision
      {
        if (counter == 0)
        {
          if (this->grippers[0].movingDirectionD == movingDirection::left)
          {
            differenceLeft = this->x + transformGripper_right.getOrigin().getX();
          }
          if (this->grippers[0].movingDirectionD == movingDirection::right)
          {
            differenceLeft = this->x - transformGripper_right.getOrigin().getX();
          }
        }
        ++counter;
        if (this->grippers[0].movingDirectionD == movingDirection::left)
        {
          this->x = transformGripper_right.getOrigin().getX() + differenceLeft;
        }
        if (this->grippers[0].movingDirectionD == movingDirection::right)
        {
          this->x = transformGripper_right.getOrigin().getX() + differenceLeft;
        }
        if (this->grippers[0].movingDirectionD == movingDirection::left)
        {
          this->y = transformGripper_right.getOrigin().getY() + this->marker.scale.y / 2;
        }
        if (this->grippers[0].movingDirectionD == movingDirection::right)
        {
          this->y = transformGripper_right.getOrigin().getY() - this->marker.scale.y / 2;
        }
      }
      if (isGripperRightCollision(transformGripper_left, transformGripper_right))
      {
        if (lCounter == 0)
        {
          differenceRight = this->x - transformGripper_left.getOrigin().getX(); //checks left collision
          differenceRightTest = transformGripper_left.getOrigin().getY() - this->y;
        }
        ++lCounter;
        this->x = transformGripper_left.getOrigin().getX() + differenceRight;
        this->y = transformGripper_left.getOrigin().getY() + differenceRightTest;
      }
    }

    if (ros::Time::now().toSec() - timeSinceStart >= (64 + 15))
    {
      this->x = 0.19;
      this->y = 0;
      this->z = 0;
      this->dropped = false;
      this->timeSinceStart = ros::Time::now().toSec();
    }

    if (transformGripper_right.getOrigin().getZ() && transformGripper_left.getOrigin().getZ() >= 0.2 && this->z <= 0)
    {
      //this->dropped = false;
      counter = 0;
      lCounter = 0;
    }
    if (this->stucked == false && (this->z > 0))
    {
      this->dropped = true;
      this->z = this->z - 0.000005;
    }

    lastRightX = transformGripper_right.getOrigin().getX();
    lastRightY = transformGripper_right.getOrigin().getY();
    lastRightZ = transformGripper_right.getOrigin().getZ();
    lastLeftX = transformGripper_left.getOrigin().getX();
    lastLeftY = transformGripper_left.getOrigin().getY();
    lastLeftZ = transformGripper_left.getOrigin().getZ();

    lastRightYvals.push_back(lastRightY);

    this->displayMarker();
  }
}

void Marker::Marker::displayMarker()
{
  this->marker.header.frame_id = "/base_link";
  this->marker.header.stamp = ros::Time::now();
  this->marker.ns = this->name;
  this->marker.id = 0;
  this->marker.type = this->shape;
  this->marker.action = visualization_msgs::Marker::ADD;
  this->marker.pose.position.x = this->x;
  this->marker.pose.position.y = this->y;
  this->marker.pose.position.z = this->z;
  this->marker.pose.orientation.x = this->oX;
  this->marker.pose.orientation.y = this->oY;
  this->marker.pose.orientation.z = this->oZ;
  this->marker.pose.orientation.w = this->oW;

  this->marker.scale.x = 0.03;
  this->marker.scale.y = 0.03;
  this->marker.scale.z = 0.03;

  this->marker.color.r = 1.0f;
  this->marker.color.g = 0.0f;
  this->marker.color.b = 0.0f;
  this->marker.color.a = 1.0;
  this->marker.lifetime = ros::Duration();
  this->marker_pub.publish(this->marker);
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
}

void Marker::Marker::setTimeSinceStart(double time)
{
  this->timeSinceStart = time;
}

void Marker::Marker::createGrippers()
{
  Gripper gripperRight = {"gripperRight", movingDirection::notMoving};
  this->grippers.push_back(gripperRight);
  Gripper gripperLeft = {"gripperleft", movingDirection::notMoving};
  this->grippers.push_back(gripperLeft);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker");
  std::cout << "MARKER" << std::endl;
  Marker::Marker m(0.19, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, "cyllinder1");
  sleep(5);
  m.setTimeSinceStart(ros::Time::now().toSec());
  m.displayMarker();
  m.createGrippers();
  m.markerCallback();

  ros::spin();
  // Set our initial shape type to be a cube
}
