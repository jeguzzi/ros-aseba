cmake_minimum_required(VERSION 3.0.2)
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

if(WIN32)
  # avoid to overwrite std::max in dashel-win32.cpp
  add_compile_definitions(NOMINMAX)
  # force it to build .lib
  option(BUILD_SHARED_LIBS "" OFF)
endif()


find_package(catkin REQUIRED COMPONENTS
	roscpp
	asebaros_msgs
  diagnostic_updater
)

# find_package(catkin COMPONENTS diagnostic_updater)
if(diagnostic_updater_FOUND )
  set(EXTRA_DEPS diagnostic_updater)
  message("Will use ROS diagnostics")
else()
  message("Will not use ROS diagnostics")
endif()
message("Using ROS VERSION $ENV{ROS_VERSION}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS thread)
find_package(LibXml2 REQUIRED)
# Else catkin [DEPENDS] complain
set(LibXml2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR})
set(LibXml2_LIBRARIES ${LIBXML2_LIBRARIES})

macro(codesign target)
endmacro(codesign)

if(DEFINED ENV{DASHEL_DIR})
  set(DASHEL_DIR $ENV{DASHEL_DIR})
else()
  set(DASHEL_DIR ${PROJECT_SOURCE_DIR}/dashel)
endif()

add_subdirectory(${DASHEL_DIR} dashel)

if(DEFINED ENV{ASEBA_DIR})
  set(ASEBA_DIR $ENV{ASEBA_DIR})
else()
  set(ASEBA_DIR ${PROJECT_SOURCE_DIR}/aseba)
endif()

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
  CATKIN_DEPENDS roscpp asebaros_msgs ${EXTRA_DEPS}
  DEPENDS LibXml2
)

###########
## Build ##
###########

message("catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ASEBA_DIR}/aseba
  ${LIBXML2_INCLUDE_DIR}
)

add_executable(asebaros
  src/aseba_script.cpp
  src/aseba_dashel_hub.cpp
  src/asebaros_node.cpp
  src/asebaros_node_ros1.cpp
  src/asebaros_node_manager.cpp
  src/asebaros_node_manager_ros1.cpp)

if(diagnostic_updater_FOUND )
  target_compile_definitions(asebaros PUBLIC DIAGNOSTICS)
endif()
target_compile_definitions(asebaros PUBLIC ROS_VERSION_M=$ENV{ROS_VERSION})

target_link_libraries(
  asebaros
  dashel asebacommon asebacompiler asebadashelplugins
  ${catkin_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  )



#############
## Install ##
#############

# THIS DOES NOT WORK AND IS NOT NEEDED IN CMAKE<=3.13
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.13")
  install(TARGETS dashel asebacommon asebacompiler asebadashelplugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endif ()

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
