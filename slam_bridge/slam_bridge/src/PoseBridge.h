#ifndef POSE_BRIDGE_H__
#define POSE_BRIDGE_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> 	// position 

class PoseBridge
{
	public:

	  	PoseBridge();
	  	~PoseBridge();

	protected:

	  	ros::NodeHandle node_;

	  	// subscribers
	  	ros::Subscriber outPoseSubscriber_;
		/* 
		parameters : 
			1- Network 
	  		2- Position 
  		*/
  		int p_net_port_; 
  		std::string p_hostname_;
  		
  		std::string p_out_pose_topic_;
  		int p_pose_subscriber_queue_size_;

  		// callbacks
  		void poseCallback(const geometry_msgs::PoseStamped& pose);

		bool is_network_initialized_;

  	private:
  		int sockfd_;
};


#endif // POSE_BRIDGE_H__