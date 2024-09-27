cmake_minimum_required(VERSION 3.8)
project(energirobotter_interfaces)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 20)

############################################################################

# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

############################################################################

# Interfaces


ament_export_dependencies(rosidl_default_runtime)
ament_package()
