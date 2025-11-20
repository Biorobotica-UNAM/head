cmake_minimum_required(VERSION 3.8)
project(head)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# ---- Executable ----
add_executable(head_node src/head_node.cpp)

ament_target_dependencies(head_node
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_sdk
)

# ---- Link Dynamixel SDK ----
target_link_libraries(head_node
  ${DXL_LIB_CPP}
)

# ---- Install ----
install(TARGETS head_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
