#include "SlamBridge.h"

#define LOOP_RATE 1

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "slam_bridge");
  	SlamBridge sb;
  	ros::spin();

  	return(0);
}
