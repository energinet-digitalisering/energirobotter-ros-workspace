cmake_minimum_required(VERSION 3.8)
project(image_processing)

# Default compile flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_rotate_node src/image_rotate_node.cpp)
ament_target_dependencies(image_rotate_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# On some systems you may need explicit OpenCV include/link
target_include_directories(image_rotate_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_rotate_node ${OpenCV_LIBRARIES})

install(TARGETS image_rotate_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
