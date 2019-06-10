#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <using_urdf/communicatorAction.h>
#include <fstream>
#include "client.h"

client::Client::Client()
{
}

client::Client::~Client()
{
}

std::vector<uint64_t> client::Client::parseMessage(std::string message)
{
  std::vector<uint64_t> messages;
  std::string value;
  for (char &c : message)
  {
    if (c != '#' && c != ' ' && c != '<' && c != 'P' && c != 'T')
    {
      value += c;
    }

    if (c == ' ')
    {
      messages.push_back((uint64_t)std::stoi(value));
      value = "";
    }
  }

  return messages;
}
void client::Client::CallBackMsg(const std_msgs::StringConstPtr &msg)
{
  actionlib::SimpleActionClient<using_urdf::communicatorAction> ac("AL5D", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  messages = this->parseMessage(msg->data);

  int sz = messages.size() - 1;
  int iterator = 0;
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  using_urdf::communicatorActionGoal goal;
  goal.goal.pose = 1;
  goal.goal.servo.resize(7);
  goal.goal.angle.resize(7);
  goal.goal.size = messages.size() / 2;

  for (int i = 0; i < messages.size() - 1; ++i)
  {
    if (i % 2 == 0 && i == 0 || (i % 2 == 0 && i != 0))
    {
      goal.goal.servo[iterator] = (messages[i]);
      goal.goal.angle[iterator] = (messages[i + 1]);
      iterator++;
    }
  }

  goal.goal.duration = messages[sz];

  ac.sendGoal(goal.goal);
  iterator = 0;
}

void client::Client::run()
{
  rosNode = std::make_unique<ros::NodeHandle>("rviz_simulation");
  rosSubCommands =
      rosNode->subscribe("/robot_control", 1,
                         &client::Client::CallBackMsg, this);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "client");

  client::Client c;
  c.run();
  ros::spin();
  //exit
  return 0;
}
