cmake_minimum_required(VERSION 3.8)
project(behaviour_manager)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 20)


############################################################################

# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


############################################################################

# Nodes (C++)

# behaviour_manager
add_executable(behaviour_manager nodes/behaviour_manager.cpp)
ament_target_dependencies(behaviour_manager rclcpp)

install(TARGETS
  behaviour_manager
DESTINATION lib/${PROJECT_NAME})

############################################################################


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
