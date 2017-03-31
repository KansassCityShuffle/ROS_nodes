#include "PoseBridge.h"

#define LOOP_RATE 1

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pose_bridge");
  	PoseBridge pb;
  	ros::spin();

  	return(0);
}
