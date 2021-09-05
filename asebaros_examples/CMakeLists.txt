cmake_minimum_required(VERSION 3.0.2)
project(asebaros_examples)

find_package(catkin REQUIRED COMPONENTS asebaros)

catkin_package(
  CATKIN_DEPENDS asebaros
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
