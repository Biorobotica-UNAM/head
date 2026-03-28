cmake_minimum_required(VERSION 3.8)
project(head)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(head_interfaces REQUIRED)

add_executable(head_node src/head_node.cpp)

ament_target_dependencies(head_node
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_sdk
  head_interfaces
)

target_link_libraries(head_node
  ${DXL_LIB_CPP}
)

install(TARGETS head_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
