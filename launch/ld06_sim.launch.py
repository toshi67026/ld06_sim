#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("ld06_sim")

    empty_world_path = os.path.join(pkg_dir, "worlds", "empty.world")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    urdf_file_path = os.path.join(pkg_dir, "description", "urdf", "ld06.urdf")
    rviz_config_path = os.path.join(pkg_dir, "rviz", "display.rviz")

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": empty_world_path}.items(),
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py"))
    )

    spawn_model_node = Node(
        package="ld06_sim",
        executable="spawn_model",
        arguments=[
            "--model_name",
            LaunchConfiguration("model", default="ld06"),
            "-x",
            LaunchConfiguration("x_init", default=0.5),
            "-y",
            LaunchConfiguration("y_init", default=0.5),
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[urdf_file_path],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz2_node)
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(spawn_model_node)

    return ld
