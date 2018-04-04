cmake_minimum_required(VERSION 2.8.3)
project(thymio_driver)

find_package(catkin REQUIRED COMPONENTS
	rospy
	asebaros_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	tf
	thymio_msgs
	dynamic_reconfigure
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

generate_dynamic_reconfigure_options(
  cfg/Thymio.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy asebaros_msgs std_msgs geometry_msgs tf sensor_msgs nav_msgs thymio_msgs
)

###########
## Build ##
###########


#############
## Install ##
#############
