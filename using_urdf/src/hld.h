#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <using_urdf/communicatorAction.h>
#include "std_msgs/String.h"
#include "using_urdf/lld.h"
#include <thread>

namespace hld
{
  static int processing = 0;

class hld
{
  //class State *current;

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<using_urdf::communicatorAction> as_;
  std::string action_name_;
  using_urdf::communicatorActionFeedback feedback_;
  using_urdf::communicatorActionResult result_;

private:
  class State *current;
  std::string currentStatename;
  std::string pose_name;
  bool received = false;
  

public:
  lld::lld lld;
  hld(std::string name, std::string port);
  virtual ~hld();
  using_urdf::communicatorGoalConstPtr goalptr;
  static void sendActionThread(hld *m, const using_urdf::communicatorGoalConstPtr &goal);
  void setCurrent(State *s);
  void setCurrentStatename(std::string s);
  bool detectlowlvl();
  void executeCB(const using_urdf::communicatorGoalConstPtr &goal);
  void startListening();
  void stopListening();
  void setAborted();
  void setPreempted();
  void setSucceeded();
  void publishFeedback();
  bool isPreempt();
  void setReceived(const bool inputreceived);
  const bool &getReceived();
  const std::string &getActionName();
  const std::string &getPosName();
  const std::string &getCurrentStatename();
  void setFeedback(int16_t angle, uint16_t time);
  void setResult();
};
} // namespace hld
