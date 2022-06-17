#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory("ld06_sim")

    empty_world_path = os.path.join(pkg_dir, "worlds", "empty.world")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    urdf_path = os.path.join(pkg_dir, "description", "urdf", "ld06.urdf")
    rviz_config_path = os.path.join(pkg_dir, "rviz", "display.rviz")

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": empty_world_path}.items(),
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py"))
    )

    spawn_ld06_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "ld06", "-database", "ld06", "-x", "0.5", "-y", "0.5"],
    )

    spawn_unit_box_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "unit_box", "-database", "unit_box", "-x", "1.5", "-y", "1.5"],
    )

    spawn_unit_cylinder_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "unit_cylinder", "-database", "unit_cylinder", "-x", "1.5", "-y", "0"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[urdf_path],
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.5", "0.5", "0", "0", "0", "0", "world", "base_link"],
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

    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(spawn_ld06_node)
    ld.add_action(spawn_unit_box_node)
    ld.add_action(spawn_unit_cylinder_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(rviz2_node)

    return ld
