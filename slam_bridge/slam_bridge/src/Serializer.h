#ifndef SERIALIZER_H__
#define SERIALIZER_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h> 

class Serializer 
{
	public: 
		static int addZeroPadding(std::string *header);
		static int checkDifferences(const nav_msgs::Path& path, const nav_msgs::Path& old_path);
		static int serializePose(const geometry_msgs::PoseStamped& pose, std::string *buffer);
		static int serializeIniMap(const nav_msgs::OccupancyGrid& map, std::string *buffer);
		static int serializeMap(const nav_msgs::OccupancyGrid& map, const nav_msgs::OccupancyGrid& old_map, std::string *buffer);
		static int serializeTrajectory(const nav_msgs::Path& path, const nav_msgs::Path& old_path, std::string *buffer);
		static int serializeIniTrajectory(const nav_msgs::Path& path, std::string *buffer);

	private: 
		Serializer(){}
};

#endif // SERIALIZER_H__