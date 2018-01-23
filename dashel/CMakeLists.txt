cmake_minimum_required(VERSION 2.8.3)
project(dashel_ros)

find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS upstream_src/
  LIBRARIES dashel
)

###########
## Build ##
###########

include_directories(
  upstream_src/dashel/
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(upstream_src)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS dashel
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY upstream_src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
