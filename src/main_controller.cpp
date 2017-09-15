#include "JoystickNode.h"
#include "Helper.h"
#include "ros/ros.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "rl_nav");
	Helper helper;
	JoystickNode joyNode;
	ros::spin();
	return 0;
}
