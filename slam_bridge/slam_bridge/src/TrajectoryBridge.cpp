#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "TrajectoryBridge.h"
#include "Serializer.h"

#define LOOP_RATE 1

void error(const char *msg) 
{
	std::cout << "error" << std::endl;
    std::cout << msg << std::endl;
    exit(-1);
}

TrajectoryBridge::TrajectoryBridge()
{
	std::cout << "starting trajectory bridge" << std::endl;
	
	is_network_initialized_ = false;
 	is_path_initialized_ = false; 
	ros::NodeHandle private_nh_("~");
	ros::Rate loop_rate(LOOP_RATE);
	
	/************************************/
	/* Retreive ROS server's parameters */
	/************************************/

	// network 
	private_nh_.param("net_port", p_net_port_, 1234); 
	private_nh_.param("hostname", p_hostname_, std::string("127.0.0.1"));
	// position 
	private_nh_.param("trajectory_pose_topic", p_trajectory_topic_, std::string("trajectory"));
	private_nh_.param("trajectory_subscriber_queue_size", p_trajectory_subscriber_queue_size_, 5);

	/****************************/
	/* Initialize subscriptions */
	/****************************/

	trajectorySubscriber_ = node_.subscribe(p_trajectory_topic_, p_trajectory_subscriber_queue_size_, 
											&TrajectoryBridge::trajectoryCallback, this);

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


TrajectoryBridge::~TrajectoryBridge()
{

}

void TrajectoryBridge::trajectoryCallback(const nav_msgs::Path& path)
{

	ros::WallTime startTime = ros::WallTime::now();	
 	
 	if(!is_network_initialized_)
 	{
 		error("error network is not initialized in trajectoryCallback");
 		return;
 	}

 	std::string buffer; 

 	if( ! is_path_initialized_ )
 	{
 		if( Serializer::serializeIniTrajectory(path, &buffer) < 0 )
 				error("error while serializing initial path in trajectoryCallback"); 
 			is_path_initialized_ = true;
 	}
 	else
	 	if( Serializer::serializeTrajectory(path, old_path_, &buffer) < 0)
	 		error("error while serializing Path in trajectoryCallback");
	
	old_path_ = path; 
 	if( write(sockfd_, buffer.c_str(), buffer.length()) < 0 )
 			error("error writting to socket in trajectoryCallback");
}