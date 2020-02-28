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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asebaros_msgs REQUIRED)
find_package(LibXml2 REQUIRED)

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

set(ASEBA_DIR $ENV{ASEBA_DIR})  #/Users/Jerome/Dev/Others/aseba
set(DASHEL_DIR $ENV{DASHEL_DIR}) #/Users/Jerome/Dev/Others/dashel

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
add_subdirectory(${ASEBA_DIR}/aseba/compiler asebacommon)
add_subdirectory(${ASEBA_DIR}/aseba/common asebacompiler)
add_subdirectory(${ASEBA_DIR}/aseba/transport/dashel_plugins dashel_plugins)

add_executable(asebaros src/asebaros.cpp)
ament_target_dependencies(asebaros rclcpp std_msgs asebaros_msgs)
target_link_libraries(asebaros dashel asebacommon asebacompiler asebadashelplugins ${LIBXML2_LIBRARIES})

target_compile_options(asebaros PRIVATE -Wno-literal-suffix)

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
