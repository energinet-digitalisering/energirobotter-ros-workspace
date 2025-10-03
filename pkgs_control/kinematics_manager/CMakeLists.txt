cmake_minimum_required(VERSION 3.8)
project(kinematics_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # allows mixing Python + C++
find_package(rclcpp REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Make headers available
include_directories(include)

# Build library 
add_library(trac_ik_manager src/trac_ik_manager.cpp)
ament_target_dependencies(trac_ik_manager trac_ik_lib)

# C++ node
add_executable(trac_ik_node nodes/trac_ik_node.cpp)
ament_target_dependencies(trac_ik_node rclcpp geometry_msgs sensor_msgs)
target_link_libraries(trac_ik_node trac_ik_manager)

add_executable(reachability_map_generator nodes/reachability_map_generator.cpp)
ament_target_dependencies(reachability_map_generator rclcpp)
target_link_libraries(reachability_map_generator trac_ik_manager)

# Install targets
install(TARGETS
  trac_ik_manager 
  trac_ik_node 
  reachability_map_generator
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY 
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
