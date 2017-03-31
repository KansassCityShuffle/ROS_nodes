#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "MapBridge.h"
#include "Serializer.h"

#define LOOP_RATE 1

void error(const char *msg) 
{
	std::cout << "error" << std::endl;
    std::cout << msg << std::endl;
    exit(-1);
}

MapBridge::MapBridge()
{
	std::cout << "starting map bridge" << std::endl;
	
	is_network_initialized_ = false;
	is_map_initialized_ = false; 
 
	ros::NodeHandle private_nh_("~");
	ros::Rate loop_rate(LOOP_RATE);
	
	/************************************/
	/* Retreive ROS server's parameters */
	/************************************/

	// network 
	private_nh_.param("net_port", p_net_port_, 12345); 
	private_nh_.param("hostname", p_hostname_, std::string("127.0.0.1"));
	// map
	private_nh_.param("map_topic", p_map_topic_, std::string("map"));
	private_nh_.param("map_subscriber_queue_size", p_map_subscriber_queue_size_, 5);

	/****************************/
	/* Initialize subscriptions */
	/****************************/

	mapSubscriber_ = node_.subscribe(p_map_topic_, p_map_subscriber_queue_size_, &MapBridge::mapCallback, this);

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

	if(connect(sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		error("error connecting server");
	
	is_network_initialized_ = true;
}

MapBridge::~MapBridge()
{

}

void MapBridge::mapCallback(const nav_msgs::OccupancyGrid& map)
{

 	ros::WallTime startTime = ros::WallTime::now();	
 	
 	if(!is_network_initialized_)
 	{
 		error("error network is not initialized in mapCallback");
 		return;
 	}

 	std::string buffer; 

 	if (!is_map_initialized_)
 	{
 		if ( Serializer::serializeIniMap(map, &buffer) < 0 )
 			error("error serializing initial OccupancyGrid in mapCallabck");
 		is_map_initialized_ = true;
 	}
 	else	 	
	 	if ( Serializer::serializeMap(map, old_map_, &buffer) < 0 )
	 		error("error serializing OccupancyGrids in mapCallback");

 	old_map_ = map;   
 	if( write(sockfd_, buffer.c_str(), buffer.length()) < 0 )
 			error("error writting to socket in mapCallback");
}
