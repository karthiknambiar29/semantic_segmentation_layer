cmake_minimum_required(VERSION 3.8)
project(semantic_segmentation_layer)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  geometry_msgs
  message_filters
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  vision_msgs
)

include_directories(include)

# Create the semantic segmentation layer library
add_library(semantic_segmentation_layer SHARED
  src/semantic_segmentation_layer.cpp
  src/segmentation_buffer.cpp
)

target_include_directories(semantic_segmentation_layer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Humble ships classic (non-namespaced) ament CMake config for these packages,
# so resolve them through ament_target_dependencies rather than imported targets.
ament_target_dependencies(semantic_segmentation_layer ${dependencies})

# Install library
install(TARGETS semantic_segmentation_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description, example config and launch files
install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
# fake_inputs.py is referenced by the launch file for a robot-free smoke test
install(PROGRAMS test/fake_inputs.py
  DESTINATION share/${PROJECT_NAME}/test
)

# Export plugin description file for pluginlib
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(semantic_segmentation_layer)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
