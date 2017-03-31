#include "MapBridge.h"

#define LOOP_RATE 1

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "map_bridge");
  	MapBridge pb;
  	ros::spin();

  	return(0);
}