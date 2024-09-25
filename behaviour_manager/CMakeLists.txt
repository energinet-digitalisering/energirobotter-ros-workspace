cmake_minimum_required(VERSION 3.8)
project(behaviour_manager)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 20)


############################################################################

# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

############################################################################

# Libraries

include_directories(include)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

## service_client (header only)
add_library(service_client INTERFACE)
ament_export_targets(service_client HAS_LIBRARY_TARGET)

install(
    TARGETS service_client
    EXPORT service_client
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
ament_export_include_directories(include)

## lifecycle_service_client
add_library(lifecycle_service_client src/lifecycle_service_client.cpp)
ament_target_dependencies(lifecycle_service_client rclcpp lifecycle_msgs)
target_link_libraries(lifecycle_service_client service_client)
ament_export_targets(lifecycle_service_client HAS_LIBRARY_TARGET)

install(
  TARGETS lifecycle_service_client
  EXPORT lifecycle_service_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

############################################################################

# Nodes (C++)

# behaviour_manager
add_executable(behaviour_manager nodes/behaviour_manager.cpp)
ament_target_dependencies(behaviour_manager rclcpp lifecycle_msgs)
target_link_libraries(behaviour_manager lifecycle_service_client)

add_executable(lifecycle_talker nodes/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

install(TARGETS
  behaviour_manager
  lifecycle_talker
DESTINATION lib/${PROJECT_NAME})

############################################################################


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
