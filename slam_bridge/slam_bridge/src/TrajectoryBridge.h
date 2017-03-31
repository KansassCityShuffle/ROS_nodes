#ifndef TRAJECTORY_BRIDGE_H__
#define TRAJECTORY_BRIDGE_H__

#include <ros/ros.h>
#include <nav_msgs/Path.h> 	// trajectory 

class TrajectoryBridge
{
	public:

	  	TrajectoryBridge();
	  	~TrajectoryBridge();

	protected:

	  	ros::NodeHandle node_;

	  	// subscribers
	  	ros::Subscriber trajectorySubscriber_;
		/* 
		parameters : 
			1- Network 
	  		2- Position 
  		*/
  		int p_net_port_; 
  		std::string p_hostname_;
  		
  		std::string p_trajectory_topic_;
  		int p_trajectory_subscriber_queue_size_;

  		// callbacks
		nav_msgs::Path old_path_;
  		void trajectoryCallback(const nav_msgs::Path& path);

		bool is_network_initialized_;
		bool is_path_initialized_;

  	private:
  		int sockfd_;
};


#endif // TRAJECTORY_BRIDGE_H__