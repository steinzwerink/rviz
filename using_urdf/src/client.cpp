#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <using_urdf/communicatorAction.h>
#include <fstream>
#include "client.h"

client::client::client()
{
}

client::client::~client()
{
}

std::vector<uint64_t> client::client::parseMessage(std::string message)
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

int main(int argc, char **argv)
{
  std::cout << argv[1] << std::endl;
  //std::cout << argv[2] << std::endl;
  //std::cout << argv[3] << std::endl;

  client::client c;
  std::vector<uint64_t> parsedMessage = c.parseMessage(argv[1]);
  int sz = parsedMessage.size() - 1;
  int iterator = 0;

  ros::init(argc, argv, "client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<using_urdf::communicatorAction> ac("AL5D", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  using_urdf::communicatorActionGoal goal;
  goal.goal.pose = 1;
  goal.goal.servo.resize(7);
  goal.goal.angle.resize(7);
  goal.goal.size = parsedMessage.size() / 2;

  for (int i = 0; i < parsedMessage.size() - 1; ++i)
  {
    if (i % 2 == 0 && i == 0 || (i % 2 == 0 && i != 0))
    {
      goal.goal.servo[iterator] = (parsedMessage[i]);
      goal.goal.angle[iterator] = (parsedMessage[i + 1]);
      iterator++;
    }
  }

  goal.goal.duration = parsedMessage[sz];
  std::cout<<"SZ: "<<parsedMessage[sz]<<std::endl;
  ac.sendGoal(goal.goal);
  iterator = 0;

  //exit
  return 0;
}
