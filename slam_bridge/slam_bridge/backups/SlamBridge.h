#ifndef SLAM_BRIDGE_H__
#define SLAM_BRIDGE_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> 	// position 
#include <nav_msgs/OccupancyGrid.h> 	// map

class SlamBridge
{
	public:

	  	SlamBridge();
	  	~SlamBridge();

	protected:

	  	ros::NodeHandle node_;

	  	// subscribers
	  	ros::Subscriber outPoseSubscriber_;
		ros::Subscriber mapSubscriber_; 

		/* 
		parameters : 
			1- Network 
	  		2- Position 
	  		3- Map 
  		*/
  		int p_net_port_; 
  		std::string p_hostname_;
  		
  		std::string p_out_pose_topic_;
  		int p_pose_subscriber_queue_size_;
  		
  		std::string p_map_topic_;
  		int p_map_subscriber_queue_size_; 
  		
  		// callbacks
  		void poseCallback(const geometry_msgs::PoseStamped& pose);
		void mapCallback(const nav_msgs::OccupancyGrid& map);	

		bool is_network_initialized_;

  	private:
  		int sockfd_;
};


#endif // SLAM_BRIDGE_H__