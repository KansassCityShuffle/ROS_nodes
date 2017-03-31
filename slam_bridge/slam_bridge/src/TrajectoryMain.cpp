#include "TrajectoryBridge.h"

#define LOOP_RATE 1

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "trajectory_bridge");
  	TrajectoryBridge tb;
  	ros::spin();

  	return(0);
}
