cmake_minimum_required(VERSION 2.8.3)
project(thymio_description)

find_package(catkin REQUIRED COMPONENTS
	urdf
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS urdf
)

###########
## Build ##
###########


#############
## Install ##
#############
