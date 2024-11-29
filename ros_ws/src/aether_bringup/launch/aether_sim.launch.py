import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import em


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("aether_bringup")
    pkg_project_gazebo = get_package_share_directory("aether_gazebo")
    pkg_project_description = get_package_share_directory("aether_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Load the SDF file from "description" package
    template_path = os.path.join(
        pkg_project_description, "models", "aether", "model.sdf.em"
    )
    with open(template_path) as template_file:
        template = template_file.read()
    robot_desc = em.expand(template, {})

    # Setup to launch the simulator and Gazebo world
    world_path = os.path.join(pkg_project_gazebo, "worlds", "simple_maze.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}",
        }.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[
            {
                "name": "aether",
                "string": robot_desc,
                "z": 0.03,
            }
        ],
        output="screen",
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "config", "config.rviz"),
        ],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "gz_ros_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Convert LiDAR scans to range measurements
    tof_names = [
        "tof_right_side",
        "tof_right_diag",
        "tof_right_front",
        "tof_left_front",
        "tof_left_diag",
        "tof_left_side",
    ]
    lidar_to_range = Node(
        package="aether_gazebo",
        executable="lidar_to_range",
        output="screen",
        parameters=[
            {
                "lidar_topics": [f"/aether/{name}/scan" for name in tof_names],
                "range_topics": [f"/aether/{name}/range" for name in tof_names],
                "fov": math.radians(18),
            }
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            spawn,
            bridge,
            robot_state_publisher,
            rviz,
            lidar_to_range,
        ]
    )
