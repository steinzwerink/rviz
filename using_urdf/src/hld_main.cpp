#include "hld.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hld");

  hld::hld hld("AL5D", "/dev/ttyUSB0");
  sleep(5);
  hld.lld.initialize();

  ros::spin();
  //exit
  return 0;
}