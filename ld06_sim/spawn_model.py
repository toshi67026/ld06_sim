#!/usr/bin/env python3

import argparse
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description="Spawn obstacle into Gazebo")

    parser.add_argument("-n", "--model_name", type=str, default="ld06", help="Name of the model to spawn")
    parser.add_argument("-ns", "--namespace", type=str, default="", help="ROS namespace to apply to the plugins")
    parser.add_argument("-x", type=float, default=0, help="the x component of the initial position [meters]")
    parser.add_argument("-y", type=float, default=0, help="the y component of the initial position [meters]")
    parser.add_argument("-z", type=float, default=0, help="the z component of the initial position [meters]")

    parser.add_argument(
        "-k",
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait. Block until the future is complete if negative. Don't wait if 0.",
    )

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info("Creating Service client to connect to `/spawn_entity`")
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    node.get_logger().info(f"model_name: {args.model_name}")
    node.get_logger().info(f"namespace :{args.namespace}")
    node.get_logger().info(f"initial_position: {args.x}, {args.y}, {args.z}")

    sdf_file_path = os.path.join(get_package_share_directory("ld06_sim"), "models", str(args.model_name), "model.sdf")

    tree = ET.parse(sdf_file_path)
    root = tree.getroot()

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.model_name
    request.xml = ET.tostring(root, encoding="unicode")
    request.robot_namespace = args.namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)

    if future.result() is not None:
        print("response: %r" % future.result())
    else:
        raise RuntimeError("exception while calling service: %r" % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
