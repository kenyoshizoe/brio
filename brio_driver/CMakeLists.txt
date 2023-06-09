cmake_minimum_required(VERSION 3.5)
project(brio_driver)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(brio_msgs REQUIRED)

# add_subdirectory(proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include(FindProtobuf)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/main.proto)

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

add_library(${PROJECT_NAME}
  SHARED
  ${PROTO_SRCS}
  src/brio_driver.cc
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PROTOBUF_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARIES}
  Threads::Threads
  protobuf
  protobuf::libprotobuf
  protobuf::libprotoc
)

add_executable(brio_driver_node
  src/brio_driver_node.cc
)
target_include_directories(brio_driver_node PUBLIC
  include
  ${PROTOBUF_INCLUDE_DIRS}
)
target_link_libraries(brio_driver_node
  ${PROJECT_NAME}
)
ament_target_dependencies(brio_driver rclcpp sensor_msgs brio_msgs)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  brio_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
