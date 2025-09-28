cmake_minimum_required(VERSION 3.5)
project(assignment1_rt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtleInfo.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)


# Install Python package (so it can be imported if needed)
#ament_python_install_package(${PROJECT_NAME})

# Install Python executable nodes
install(PROGRAMS
  scripts/ui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

