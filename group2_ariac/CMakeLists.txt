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
find_package(ariac_msgs REQUIRED)


#--------
# C++
#--------
include_directories(include)

add_executable(retrieve_orders src/retrieve_orders.cpp)
ament_target_dependencies(retrieve_orders 
                          rclcpp 
                          ariac_msgs)

install(TARGETS
        retrieve_orders
        DESTINATION lib/${PROJECT_NAME}
        )


#--------
# Python
#--------
# Install Python modules
ament_python_install_package(${PROJECT_NAME} 
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_package()
