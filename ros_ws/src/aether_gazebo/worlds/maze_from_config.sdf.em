<?xml version="1.0" ?>
@{
# inputs: maze_path
import math
from aether_description.sdf_utils import decode_maze_from_text

maze = decode_maze_from_text(maze_path)
width = maze.width
height = maze.height
horizontal_walls = maze.horizontal_walls
vertical_walls = maze.vertical_walls

cell_size = 0.18
}@
<sdf version="1.8">
    <world name="demo">
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin
            filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>

        <light name="sun" type="directional">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>10 10</size>
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>10 10</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.1 0.1 0.1 1</ambient>
                        <diffuse>0.1 0.1 0.1 1</diffuse>
                        <specular>0.1 0.1 0.1 1</specular>
                    </material>
                </visual>
            </link>
        </model>

@{
post_template = """
        <include>
            <name>post_{post_idx}</name>
            <pose>{x} {y} 0 0 0 0</pose>
            <uri>package://aether_description/models/maze_post</uri>
        </include>
"""
post_idx = 0
for row in range(height + 1):
    for col in range(width + 1):
        x = row * cell_size
        y = -col * cell_size
        print(post_template.format(post_idx=post_idx, x=x, y=y))
        post_idx += 1

wall_idx = 0
wall_template = """
        <include>
            <name>wall_{wall_idx}</name>
            <pose>{x} {y} 0 0 0 {angle}</pose>
            <uri>package://aether_description/models/maze_wall</uri>
        </include>
"""
# horizontal walls
for row in range(height + 1):
    for col in range(width):
        if horizontal_walls[row][col]:
            x = row * cell_size
            y = -col * cell_size - cell_size / 2
            print(wall_template.format(wall_idx=wall_idx, x=x, y=y, angle=math.pi / 2))
            wall_idx += 1

# vertical walls
for row in range(height):
    for col in range(width + 1):
        if vertical_walls[row][col]:
            x = row * cell_size + cell_size / 2
            y = -col * cell_size
            print(wall_template.format(wall_idx=wall_idx, x=x, y=y, angle=0))
            wall_idx += 1
}@
    </world>
</sdf>
