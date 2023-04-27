cmake_minimum_required(VERSION 3.5)
project(brio_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  PATTERN ".svn" EXCLUDE
)

ament_package()
