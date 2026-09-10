# Pinhole projection helper (no ROS runtime needed)
ament_add_gtest(test_camera_projection
  test_camera_projection.cpp
)
target_link_libraries(test_camera_projection
  semantic_segmentation_layer
)
ament_target_dependencies(test_camera_projection
  sensor_msgs
)

# End-to-end LiDAR-projection buffering path
ament_add_gtest(test_projection_buffer
  test_projection_buffer.cpp
)
target_link_libraries(test_projection_buffer
  semantic_segmentation_layer
)
ament_target_dependencies(test_projection_buffer
  geometry_msgs
  nav2_util
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
