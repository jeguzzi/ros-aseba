cmake_minimum_required(VERSION 2.8.3)
project(aseba)

find_package(catkin
	REQUIRED COMPONENTS dashel_ros
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS upstream_src
  LIBRARIES asebacompiler asebacommon
  CATKIN_DEPENDS dashel_ros
)

###########
## Build ##
###########

include_directories(
  upstream_src/common
  upstream_src/compiler
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(upstream_src)

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
install(DIRECTORY upstream_src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
