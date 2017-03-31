#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "SlamBridge.h"
#include "Serializer.h"

#define LOOP_RATE 1

void error(const char *msg) 
{
	std::cout << "error" << std::endl;
    std::cout << msg << std::endl;
    //exit(0);
}

SlamBridge::SlamBridge()
{
	std::cout << "starting slam bridge" << std::endl;
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
	// map
	private_nh_.param("map_topic", p_map_topic_, std::string("map"));
	private_nh_.param("map_subscriber_queue_size", p_map_subscriber_queue_size_, 5);

	/****************************/
	/* Initialize subscriptions */
	/****************************/

	// TODO : pass queue size to 1 since rviz is not needed 
	mapSubscriber_ = node_.subscribe(p_map_topic_, p_map_subscriber_queue_size_, &SlamBridge::mapCallback, this);
	outPoseSubscriber_ = node_.subscribe(p_out_pose_topic_, p_pose_subscriber_queue_size_, &SlamBridge::poseCallback, this);

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
	else 
		is_network_initialized_ = true;
}


SlamBridge::~SlamBridge()
{

}

void SlamBridge::poseCallback(const geometry_msgs::PoseStamped& pose)
{
 	ros::WallTime startTime = ros::WallTime::now();	
 	char buffer[256] = {0}; 

 	if(is_network_initialized_)
 	{
 		memset(buffer, 0, 256); 
	 	Serializer::serializePose(pose, buffer);
	 	// debug
	 	std::cout << "buffer " << buffer << std::endl; 
	 	
	 	int n = write(sockfd_, buffer, strlen(buffer));
	 	if(n < 0)
	 		error("error writing to socket");
 	}
 	else
 		error("error network is not initialized");
}

void SlamBridge::mapCallback(const nav_msgs::OccupancyGrid& map)
{
 	ros::WallTime startTime = ros::WallTime::now();	

 	if(is_network_initialized_)
 	{
	 	char* buffer = Serializer::serializeMap(map);
	 	// debug
	 	std::cout << "buffer " << buffer << std::endl; 
	 	
	 	int n = write(sockfd_, buffer, strlen(buffer));
	 	if(n < 0)
	 		error("error writing to socket");
 	}
 	else
 		error("error network is not initialized");
}
