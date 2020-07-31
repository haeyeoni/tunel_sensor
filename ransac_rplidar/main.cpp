#include "cmd_vel_test.h"
#include "ransac_class.h"
#include "command_robot.h"

using namespace RANSAC;
using namespace COMMAND;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
  LineExtractRP ransac;
  Command command;
  ros::spin();
  return 0;
}
