cmake_minimum_required(VERSION 3.5)
project(thymio_description)

find_package(urdf REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
