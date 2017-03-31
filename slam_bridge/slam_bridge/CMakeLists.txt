cmake_minimum_required(VERSION 2.8.3)
project(slam_bridge)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-g)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs nav_msgs rospy)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs rospy std_msgs nav_msgs
)

###########
## Build ##
###########
## Declare a C++ executable

include_directories( ${catkin_INCLUDE_DIRS} src )

add_executable(trajectory_bridge
  src/TrajectoryBridge.cpp
  src/TrajectoryMain.cpp
  src/Serializer.cpp
)

add_executable(pose_bridge
  src/PoseBridge.cpp
  src/PoseMain.cpp
  src/Serializer.cpp
)

add_executable(map_bridge
  src/MapBridge.cpp
  src/MapMain.cpp
  src/Serializer.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_bridge
   ${catkin_LIBRARIES}
  )

target_link_libraries(map_bridge
   ${catkin_LIBRARIES}
  )

target_link_libraries(trajectory_bridge 
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
