cmake_minimum_required(VERSION 3.0.2)
project(darknet_msg)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
)

add_message_files(
  FILES
  CNNDetection.msg
  CNNDetectionList.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
   darknet_msg
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

