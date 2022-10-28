cmake_minimum_required(VERSION 3.0.2)
project(darknet_udp_bridge)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  darknet_msg
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES beginner
  #CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib roscpp nodelet
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
 include/${PROJECT_NAME}
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/darknet_udp_bridge_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
