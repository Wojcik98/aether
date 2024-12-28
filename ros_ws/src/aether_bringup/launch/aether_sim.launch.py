import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node

import em
import yaml


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("aether_bringup")
    pkg_project_app = get_package_share_directory("aether_app")
    pkg_project_gazebo = get_package_share_directory("aether_gazebo")
    pkg_project_description = get_package_share_directory("aether_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Load the SDF template from "description" package
    robot_config_path = os.path.join(pkg_project_bringup, "config", "robot_config.yaml")
    with open(robot_config_path) as robot_config_file:
        robot_config = yaml.safe_load(robot_config_file)
    template_path = os.path.join(
        pkg_project_description, "models", "aether", "model.sdf.em"
    )
    with open(template_path) as template_file:
        template = template_file.read()
    robot_desc = em.expand(template, {"config": robot_config})

    # Setup to launch the simulator and Gazebo world
    # world_path = os.path.join(pkg_project_gazebo, "worlds", "maze_test.sdf")
    world_path = "/tmp/maze.sdf"
    maze_config_path = os.path.join(
        pkg_project_bringup, "config", "mazes", "simple.txt"
    )
    path_config_path = os.path.join(
        pkg_project_bringup, "config", "paths", "simple.txt"
    )
    world_template_path = os.path.join(
        pkg_project_gazebo, "worlds", "maze_from_config.sdf.em"
    )
    with open(world_template_path) as world_template_file:
        world_template = world_template_file.read()
    world_desc = em.expand(world_template, {"maze_path": maze_config_path})
    with open(world_path, "w") as world_file:
        world_file.write(world_desc)
    gazebo_config_path = os.path.join(pkg_project_bringup, "config", "gazebo.config")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_path} --gui-config {gazebo_config_path}",
        }.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[
            {
                "name": "aether",
                "string": robot_desc,
                "x": robot_config["starting_pose"][0],
                "y": robot_config["starting_pose"][1],
                "z": 0.0,
                "yaw": robot_config["starting_pose"][2],
            }
        ],
        output="screen",
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            f"{robot_config["starting_pose"][0]}",
            f"{robot_config["starting_pose"][1]}",
            "0",
            "0",
            "0",
            f"{robot_config["starting_pose"][2]}",
            "map",
            "odom",
        ],
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    map_visualizer = Node(
        package="aether_app",
        executable="map_visualizer",
        output="screen",
        parameters=[
            {"map_path": maze_config_path},
            {"use_sim_time": True},
        ],
    )

    path_visualizer = Node(
        package="aether_app",
        executable="path_visualizer",
        output="screen",
        parameters=[
            {"path_path": path_config_path},
            {"use_sim_time": True},
        ],
    )

    sensors_sync_component = ComposableNode(
        package="aether_app",
        plugin="SensorsSync",
        name="sensors_sync",
        namespace="aether",
        remappings=[
            ("odom", "/aether/odom"),
            ("imu", "/aether/imu"),
            ("tof_right_side", "/aether/tof_right_side/range"),
            ("tof_right_diag", "/aether/tof_right_diag/range"),
            ("tof_right_front", "/aether/tof_right_front/range"),
            ("tof_left_front", "/aether/tof_left_front/range"),
            ("tof_left_diag", "/aether/tof_left_diag/range"),
            ("tof_left_side", "/aether/tof_left_side/range"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    mapped_localization_component = ComposableNode(
        package="aether_app",
        plugin="MappedLocalizationNode",
        name="mapped_localization",
        namespace="aether",
        parameters=[
            {"map_path": maze_config_path},
            {"use_sim_time": True},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    mapped_localization_container = ComposableNodeContainer(
        name="mapped_localization_container",
        namespace="aether",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            sensors_sync_component,
            mapped_localization_component,
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
    # TODO: make composable
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
            map_to_odom,
            map_visualizer,
            path_visualizer,
            mapped_localization_container,
            robot_state_publisher,
            rviz,
            lidar_to_range,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mapped_localization_container,
                    on_exit=EmitEvent(
                        event=Shutdown(reason="mapped_localization_container exited")
                    ),
                )
            ),
        ]
    )
