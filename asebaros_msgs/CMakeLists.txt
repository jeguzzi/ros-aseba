cmake_minimum_required(VERSION 2.8.3)
project(asebaros_msgs)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AsebaEvent.msg
  AsebaAnonymousEvent.msg
	AsebaNode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetEventId.srv
  GetEventName.srv
  GetNodeId.srv
  GetNodeList.srv
  GetNodeName.srv
  GetVariableList.srv
  GetVariable.srv
  LoadScripts.srv
	LoadScriptToTarget.srv
  SetVariable.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS std_msgs  message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


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
