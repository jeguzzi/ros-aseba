cmake_minimum_required(VERSION 3.5)
project(asebaros_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/AsebaEvent.msg"
	"msg/AsebaAnonymousEvent.msg"
)

set(srv_files
  "srv/GetEventId.srv"
	"srv/GetEventName.srv"
	"srv/GetNodeId.srv"
	"srv/GetNodeList.srv"
	"srv/GetNodeName.srv"
	"srv/GetVariableList.srv"
	"srv/GetVariable.srv"
	"srv/LoadScripts.srv"
	"srv/SetVariable.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
	${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
