#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <std_msgs/String.h>

namespace client
{
class Client
{
public:
  Client();
  virtual ~Client();
  void run();
  void CallBackMsg(const std_msgs::StringConstPtr &msg);
  std::vector<uint64_t> parseMessage(std::string message);
  std::string mParsed;
  std::vector<uint64_t> messages;
  ros::NodeHandlePtr rosNode;
  ros::Subscriber rosSubCommands;
  ros::Subscriber rosSubStop;
  ros::Publisher rosPub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  ros::ServiceServer rosService;
};
} // namespace client