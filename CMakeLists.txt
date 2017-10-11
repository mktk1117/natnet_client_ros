cmake_minimum_required(VERSION 2.8.3)
project(natnet_client_ros)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES natnet_client_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
