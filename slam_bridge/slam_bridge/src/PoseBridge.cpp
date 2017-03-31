#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "PoseBridge.h"
#include "Serializer.h"

#define LOOP_RATE 1

void error(const char *msg) 
{
	std::cout << "error" << std::endl;
    std::cout << msg << std::endl;
    exit(-1);
}

PoseBridge::PoseBridge()
{
	std::cout << "starting pose bridge" << std::endl;
	is_network_initialized_ = false;
 
	// create private node for parameters server 
	ros::NodeHandle private_nh_("~");
	ros::Rate loop_rate(LOOP_RATE);
	
	/************************************/
	/* Retreive ROS server's parameters */
	/************************************/

	// network 
	private_nh_.param("net_port", p_net_port_, 1234); 
	private_nh_.param("hostname", p_hostname_, std::string("127.0.0.1"));
	// position 
	private_nh_.param("out_pose_topic", p_out_pose_topic_, std::string("slam_out_pose"));
	private_nh_.param("pose_subscriber_queue_size", p_pose_subscriber_queue_size_, 5);

	/****************************/
	/* Initialize subscriptions */
	/****************************/

	outPoseSubscriber_ = node_.subscribe(p_out_pose_topic_, p_pose_subscriber_queue_size_, &PoseBridge::poseCallback, this);

	/********************************/
	/* Start networking through TCP */
	/********************************/

	struct sockaddr_in serv_addr; 
	struct hostent *server; 
	char buffer[256]; 

	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	
	if(sockfd_ < 0)
		error("error opening socket");
	
	server = gethostbyname(p_hostname_.c_str());
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET; 
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(p_net_port_); 

	// connect 
	if(connect(sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		error("error connecting server");
	is_network_initialized_ = true;
}


PoseBridge::~PoseBridge()
{

}

void PoseBridge::poseCallback(const geometry_msgs::PoseStamped& pose)
{
 	ros::WallTime startTime = ros::WallTime::now();	
 	std::string buffer; 

 	if(is_network_initialized_)
 	{
	 	if( Serializer::serializePose(pose, &buffer) < 0)
	 		error("error while serializing PoseStamped in poseCallback");
	 	
	 	std::cout << "buffer " << buffer << std::endl; 
	 	std::cout << "REAL seq " << pose.header.seq;
	 	if( write(sockfd_, buffer.c_str(), buffer.length()) < 0 )
	 		error("error writing to socket");
 	}
 	else
 		error("error network is not initialized");
}