cmake_minimum_required(VERSION 3.5)
project(asebaros_examples)

find_package(ament_cmake REQUIRED)
find_package(asebaros REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  simulation
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
