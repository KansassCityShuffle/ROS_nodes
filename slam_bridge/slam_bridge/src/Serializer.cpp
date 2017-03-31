#include "Serializer.h"
#include <cstdint>
#include <string>

#define HEADER_SIZE  20
#define POS_ID 		 "POS"
#define INI_MAP_ID 	 "INI_MAP"
#define MAP_ID 		 "MAP"
#define INI_TRJ_ID	 "INI_TRJ" 
#define TRJ_ID 		 "TRJ"


/**
*	This class defines a simple ROS topics Serialization protocol.
*	ROS topics are sorted in order to avoid useless content transfer from this bridge to the IHM.  
*	All functions are statics and this class can not be implemented. 
*	The resulting TCP packet will contain one of the following "headers" in it payload, accordingly to it purpose : 
* 	"POS" 		= position update
*  	"INI_MAP" 	= initialize occupancy grid 
* 	"MAP" 		= occupancy grid update		
*	"INI_TRJ"	= initialize trajectory
*  	"TRJ" 		= trajectory update	
*
*/

int Serializer::addZeroPadding(std::string *header) 
{
	if(header->length() < HEADER_SIZE)
	{	
		int h_length =  header->length();
		for(int i = 0; i < HEADER_SIZE - h_length; i ++)
			header->append("0");
		return 0; 
	}	
	else
	{
		std::cout << "Serialization error : pose header is longer than 20" << std::endl;
		return -1;
	}
}

int Serializer::serializePose(const geometry_msgs::PoseStamped& pose, std::string *buffer)
{  
	std::string header, payload; 

	payload.append( std::to_string(pose.pose.position.x) + "," 
					+ std::to_string(pose.pose.position.y) + ","
					+ std::to_string(pose.pose.position.z) + ","
					+ std::to_string(pose.pose.orientation.x) + ","
					+ std::to_string(pose.pose.orientation.y) + ","
					+ std::to_string(pose.pose.orientation.z) + ","
					+ std::to_string(pose.pose.orientation.w) );

	header.append( POS_ID );
	header.append( "," + std::to_string(payload.length()) + ",");

	if ( Serializer::addZeroPadding(&header) < 0 )
		return -1; 

	buffer->append(header);	
	buffer->append(payload);

	return 0; 
}

int Serializer::serializeIniMap(const nav_msgs::OccupancyGrid& map, std::string *buffer)
{
	std::string header, payload; 

	payload.append( std::to_string(map.info.width) + ","
					+ std::to_string(map.info.height) + ","
					+ std::to_string(map.info.resolution) + ","
					+ std::to_string(map.info.origin.position.x) + ","
					+ std::to_string(map.info.origin.position.y) + ","
					+ std::to_string(map.info.origin.position.z) + ","
					+ std::to_string(map.info.origin.orientation.x) + ","
					+ std::to_string(map.info.origin.orientation.y) + ","
					+ std::to_string(map.info.origin.orientation.z) + ","
					+ std::to_string(map.info.origin.orientation.w) ); 

	header.append( INI_MAP_ID );
	header.append( "," + std::to_string(payload.length()) + ",");

	if ( Serializer::addZeroPadding(&header) < 0 )
		return -1; 

	buffer->append(header);
	buffer->append(payload);

	return 0; 
}

int Serializer::serializeMap(const nav_msgs::OccupancyGrid& map, const nav_msgs::OccupancyGrid& old_map, std::string *buffer)
{
	std::string header, payload; 

	if(map.info.width != old_map.info.width || map.info.height != old_map.info.height)
	{
		std::cout << "Serialization error : Map and Old_map have different dimensions" << std::endl; 
		return -1; 
	}	
	
	for(int i = 0; i < map.info.width * map.info.height; i++)
	{
		if(map.data[i] != old_map.data[i])
		{
			payload.append(std::to_string(i) + ","
					+ std::to_string(map.data[i]) + "," );
		}
	}		
	payload.append( "EOP" );
	header.append( MAP_ID );
	header.append( "," + std::to_string(payload.length()) + ",");
	
	if ( Serializer::addZeroPadding(&header) < 0 )
		return -1; 
	
	buffer->append(header);
	buffer->append(payload);	

	return 0; 
}


int Serializer::checkDifferences(const nav_msgs::Path& path, const nav_msgs::Path& old_path)
{
	for(int i = 0; i < old_path.poses.size(); i++)
	{
		if(old_path.poses[i].pose.position.x != path.poses[i].pose.position.x
			|| old_path.poses[i].pose.position.y != path.poses[i].pose.position.y
			|| old_path.poses[i].pose.position.z != path.poses[i].pose.position.z)
		{
			std::cout << "send all" << std::endl;
			return -1; 
		}	 	
		std::cout << "same " << i << std::endl;
	}
}

int Serializer::serializeTrajectory(const nav_msgs::Path& path, const nav_msgs::Path& old_path, std::string *buffer)
{
	std::string header, payload; 
	if( Serializer::checkDifferences(path, old_path) < 0 )
		return -1; 
	
	if(old_path.poses.size() > path.poses.size())
	{
		std::cout << "old path is bigger than new one " << std::endl; 
		return -1;
	} 
	for(int i = 0; i < old_path.poses.size(); i ++)
	{

	}
	std::cout << "path update length : " << path.poses.size() - old_path.poses.size() << std::endl; 
	for(int i = old_path.poses.size(); i < path.poses.size(); i++)
		payload.append( std::to_string(path.poses[i].pose.position.x) + ","
						+ std::to_string(path.poses[i].pose.position.y) + ","
						+ std::to_string(path.poses[i].pose.position.z) + "," );
		
	payload.append("EOP"); 

	header.append( TRJ_ID );
	header.append( "," + std::to_string(payload.length()) + ",");

	if ( Serializer::addZeroPadding(&header) < 0 )
		return -1; 

	buffer->append(header);
	buffer->append(payload);

	return 0; 
}

int Serializer::serializeIniTrajectory(const nav_msgs::Path& path, std::string *buffer)
{
	std::string header, payload; 

	std::cout << "path initialization length : " << path.poses.size() << std::endl; 
	for(int i = 0; i < path.poses.size(); i++)
	{
		payload.append( std::to_string(path.poses[i].pose.position.x) + ","
						+ std::to_string(path.poses[i].pose.position.y) + ","
						+ std::to_string(path.poses[i].pose.position.z) + "," );
	}	
	payload.append("EOP"); 

	header.append( TRJ_ID );
	header.append( "," + std::to_string(payload.length()) + ",");

	if ( Serializer::addZeroPadding(&header) < 0 )
		return -1; 

	buffer->append(header);
	buffer->append(payload);

	return 0; 
}