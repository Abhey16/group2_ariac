cmake_minimum_required(VERSION 3.8)
project(group2_ariac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ariac_msgs REQUIRED)

# Nodes
add_executable(start_competition_node start_competition_node.cpp)
ament_target_dependencies(start_competition_node rclcpp std_srvs ariac_msgs)

add_executable(order_processing_node order_processing_node.cpp)
ament_target_dependencies(order_processing_node rclcpp ariac_msgs)

install(TARGETS
  start_competition_node
  order_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

#--------
# Python
#--------
# Install Python modules
ament_python_install_package(${PROJECT_NAME} 
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_package()
