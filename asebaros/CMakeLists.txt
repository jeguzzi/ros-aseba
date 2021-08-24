cmake_minimum_required(VERSION 3.0)
# if (POLICY CMP0048)
#   cmake_policy(SET CMP0048 NEW)
# endif (POLICY CMP0048)
project(asebaros)

# if (CMAKE_VERSION VERSION_LESS "3.1")
#   if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
#   endif ()
# else ()
#   set (CMAKE_CXX_STANDARD 14)
# endif ()


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(catkin REQUIRED COMPONENTS
	roscpp
	asebaros_msgs
  diagnostic_updater
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS thread)
find_package(LibXml2 REQUIRED)
# Else catkin [DEPENDS] complain
set(LibXml2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR})
set(LibXml2_LIBRARIES ${LIBXML2_LIBRARIES})

macro(codesign target)
endmacro(codesign)

set(ASEBA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../aseba/upstream_src)
set(DASHEL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dashel/upstream_src)

add_subdirectory(${DASHEL_DIR} dashel)

set(LIB_VERSION_MAJOR 3) # Must be bumped for incompatible ABI changes
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})
include(FeatureSummary)
include(${ASEBA_DIR}/CMakeModules/aseba_conf.cmake)
include(${ASEBA_DIR}/CMakeModules/cpp_features.cmake)

include_directories(
${ASEBA_DIR}/aseba
)

add_subdirectory(${ASEBA_DIR}/aseba/common asebacommon)
add_subdirectory(${ASEBA_DIR}/aseba/compiler asebacompiler)
add_subdirectory(${ASEBA_DIR}/aseba/transport/dashel_plugins dashel_plugins)



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
  CATKIN_DEPENDS roscpp asebaros_msgs diagnostic_updater
  DEPENDS LibXml2
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ASEBA_DIR}/aseba
  ${LIBXML2_INCLUDE_DIR}
  # ${Boost_INCLUDE_DIRS}
)

add_executable(asebaros src/asebaros.cpp)
target_link_libraries(
  asebaros
  dashel asebacommon asebacompiler asebadashelplugins
  ${catkin_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  )

#############
## Install ##
#############


install(TARGETS dashel asebacommon asebacompiler asebadashelplugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


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
