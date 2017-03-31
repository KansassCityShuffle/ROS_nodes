#ifndef MAP_BRIDGE_H__
#define MAP_BRIDGE_H__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h> 	// map

class MapBridge
{
	public:

	  	MapBridge();
	  	~MapBridge();

	protected:

	  	ros::NodeHandle node_;

	  	// subscribers
		ros::Subscriber mapSubscriber_; 

		/* 
		parameters : 
			1- Network 
	  		2- Map 
  		*/
  		int p_net_port_; 
  		std::string p_hostname_;

  		std::string p_map_topic_;
  		int p_map_subscriber_queue_size_; 
  		
  		// callbacks
		void mapCallback(const nav_msgs::OccupancyGrid& map);	

		nav_msgs::OccupancyGrid old_map_;
		bool is_network_initialized_;
		bool is_map_initialized_;

  	private:
  		int sockfd_;
};


#endif // MAP_BRIDGE_H__