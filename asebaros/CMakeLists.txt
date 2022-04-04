cmake_minimum_required(VERSION 3.5)
project(asebaros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  # avoid to overwrite std::max in dashel-win32.cpp
  add_compile_definitions(NOMINMAX)
  # force it to build .lib
  option(BUILD_SHARED_LIBS "" OFF)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asebaros_msgs REQUIRED)
find_package(LibXml2 REQUIRED)

find_package(diagnostic_msgs QUIET)
find_package(diagnostic_updater QUIET)
if( diagnostic_msgs_FOUND AND diagnostic_updater_FOUND )
  add_compile_definitions(DIAGNOSTICS)
  set(EXTRA_DEPS diagnostic_msgs diagnostic_updater)
  message("Will use ROS diagnostics")
else()
  message("Will not use ROS diagnostics")
endif()
add_compile_definitions(ROS_VERSION_M=$ENV{ROS_VERSION})
message("Using ROS VERSION $ENV{ROS_VERSION}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

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
add_subdirectory(${ASEBA_DIR}/aseba/compiler asebacompiler)
add_subdirectory(${ASEBA_DIR}/aseba/common asebacommon)
add_subdirectory(${ASEBA_DIR}/aseba/transport/dashel_plugins dashel_plugins)

add_executable(asebaros
  src/aseba_script.cpp
  src/aseba_dashel_hub.cpp
  src/asebaros_node.cpp
  src/asebaros_node_ros2.cpp
  src/asebaros_node_manager.cpp
  src/asebaros_node_manager_ros2.cpp)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   target_compile_options(asebaros PUBLIC -Wall -Wextra -Wpedantic)
# endif()


ament_target_dependencies(asebaros rclcpp std_msgs asebaros_msgs ${EXTRA_DEPS})
target_link_libraries(asebaros dashel asebacommon asebacompiler asebadashelplugins ${LIBXML2_LIBRARIES})

target_include_directories(asebaros
  PUBLIC
    ${LIBXML2_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${DASHEL_DIR}>
    $<BUILD_INTERFACE:${ASEBA_DIR}/aseba>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  asebaros
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
