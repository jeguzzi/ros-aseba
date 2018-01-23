cmake_minimum_required(VERSION 2.8.3)
project(asebaros)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	aseba
	asebaros_msgs
	message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(LibXml2 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS std_msgs roscpp aseba asebaros_msgs message_runtime
  DEPENDS Boost LibXml2
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBXML2_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(asebaros src/asebaros.cpp)
target_link_libraries(asebaros ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS beginner_tutorials beginner_tutorials_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
