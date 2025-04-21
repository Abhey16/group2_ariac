cmake_minimum_required(VERSION 3.8)
project(group2_ariac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(group2_msgs REQUIRED)

# Nodes
include_directories(include include/group2_ariac)

add_executable(start_competition_node src/start_competition_node.cpp)
ament_target_dependencies(start_competition_node rclcpp std_srvs ariac_msgs)

add_executable(retrieve_orders src/retrieve_orders.cpp)
ament_target_dependencies(retrieve_orders rclcpp ariac_msgs)

add_executable(ship_orders src/ship_orders.cpp)
ament_target_dependencies(ship_orders rclcpp ariac_msgs std_srvs)

add_executable(complete_orders src/complete_orders.cpp)
ament_target_dependencies(complete_orders rclcpp ariac_msgs std_srvs)

add_executable(end_competition_node src/end_competition_node.cpp)
ament_target_dependencies(end_competition_node rclcpp std_srvs ariac_msgs)

add_executable(tray_detector src/tray_detector.cpp)
ament_target_dependencies(tray_detector rclcpp sensor_msgs cv_bridge OpenCV  geometry_msgs tf2_ros tf2_eigen Eigen3 orocos_kdl ariac_msgs)

add_executable(bin_parts_detector src/bin_parts_detector.cpp)
ament_target_dependencies(bin_parts_detector rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs group2_msgs)

install(TARGETS
        start_competition_node
        retrieve_orders
        ship_orders
        complete_orders
        end_competition_node
        tray_detector
        bin_parts_detector
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY 
        launch
        config
        resources
        DESTINATION share/${PROJECT_NAME}
        )

# set(CMAKE_BUILD_TYPE Debug) 

#--------
# Python
#--------
# Install Python modules
ament_python_install_package(${PROJECT_NAME} 
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_package()
