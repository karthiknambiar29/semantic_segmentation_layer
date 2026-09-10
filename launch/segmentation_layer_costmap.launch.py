# Copyright 2026 robot.com
#
# Runs a standalone Nav2 costmap with the semantic segmentation layer, for
# testing the plugin without a full navigation stack.
#
#   # against your robot (its TF + sensors already running):
#   ros2 launch semantic_segmentation_layer segmentation_layer_costmap.launch.py
#
#   # fully self-contained smoke test (synthetic TF + LiDAR + mask + camera_info):
#   ros2 launch semantic_segmentation_layer segmentation_layer_costmap.launch.py \
#       publish_fake_inputs:=true
#
# The costmap is published on /costmap/costmap. It only starts publishing once a
# robot pose (TF base_link->odom) AND sensor data are available.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("semantic_segmentation_layer")
    default_params = os.path.join(
        pkg_share, "config", "segmentation_layer_local_costmap.yaml"
    )
    fake_inputs_py = os.path.join(pkg_share, "test", "fake_inputs.py")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_test_tf = LaunchConfiguration("publish_test_tf")
    publish_fake_inputs = LaunchConfiguration("publish_fake_inputs")

    def stf(name, parent, child):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            arguments=["--frame-id", parent, "--child-frame-id", child],
            condition=IfCondition(publish_test_tf),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "publish_test_tf",
                default_value="false",
                description="Publish identity odom->base_link->{rgb_camera_frame,"
                "lidar_frame} static transforms (no sensor data).",
            ),
            DeclareLaunchArgument(
                "publish_fake_inputs",
                default_value="false",
                description="Run test/fake_inputs.py: synthetic TF + LiDAR + mask "
                "+ camera_info so the costmap actually populates with no robot.",
            ),
            ExecuteProcess(
                cmd=["python3", fake_inputs_py],
                output="screen",
                condition=IfCondition(publish_fake_inputs),
            ),
            stf("tf_odom_base", "odom", "base_link"),
            stf("tf_base_cam", "base_link", "rgb_camera_frame"),
            stf("tf_base_lidar", "base_link", "lidar_frame"),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                name="costmap",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_costmap",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["costmap/costmap"],
                        # Disable bond: this standalone test bringup has no
                        # supervisor to restart the node, and a slow TF startup
                        # should not abort it.
                        "bond_timeout": 0.0,
                    }
                ],
            ),
        ]
    )
