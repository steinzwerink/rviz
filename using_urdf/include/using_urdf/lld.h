#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <thread>
#include <string>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>
#include <using_urdf/communicatorAction.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "tf2_msgs/TFMessage.h"

namespace lld
{
  struct constraints
  {
    int16_t pwmUpper = 2500;
    int16_t pwmlower = 500;
    double upperarmLower = -M_PI / 2;
    double upperarmUpper = M_PI / 2;
    double forearmLower = -4.1;
    double forearmUpper = 1;
    double wristLower = -2.51;
    double wristUpper = 2.51;
    double gripperOpen = 0.0;
    double gripperClosed = 0.015707963267948433;
  };
  
class lld
{

private:
  ros::NodeHandle n;
  float value, istart, istop, ostart, ostop;
  std::vector<double> positions{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> endVec = {0, 0, 0, 0, 0, 0, 0};
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_msgs::JointState joint_state;
  std::vector<std::string> joints = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
  constraints c;
  std::vector<std::vector<double>> constraintsA = {{-M_PI, M_PI},{c.upperarmLower, c.upperarmUpper},{c.forearmLower, c.forearmUpper}, {-M_PI, M_PI}, {c.wristLower, c.wristUpper}, {c.gripperOpen, c.gripperClosed},{c.gripperOpen, c.gripperClosed}};
  uint16_t iterator = 0;

public:
  lld();
  virtual ~lld();
  bool detectDriver();
  double remap(double value, double istart, double istop, double ostart, double ostop);
  static void sendAction(const uint16_t pose, const std::vector<uint16_t> servo,const std::vector<int16_t> angle, const uint16_t duration, const uint16_t size, lld *l);
  const std::vector<double> getPosition();
  void initialize();
  void setJoints();

};
} // namespace lld
