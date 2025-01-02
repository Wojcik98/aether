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
            <cast_shadows>false</cast_shadows>
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

        <model name="maze_border">
            <static>true</static>
@{
border_template = """
            <link name="border_{border_idx}">
                <pose>{x} {y} 0.025 0 0 {angle}</pose>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>{size} 0.012 0.05</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <cast_shadows>false</cast_shadows>
                    <geometry>
                        <box>
                            <size>{size} 0.012 0.05</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.9 0.9 1</ambient>
                        <diffuse>0.9 0.9 0.9 1</diffuse>
                    </material>
                </visual>
            </link>
"""
border_idx = 0
cell_size = 0.18
x_border_size = height * cell_size
y_border_size = width * cell_size
# east border
print(border_template.format(
    border_idx=border_idx,
    x=x_border_size / 2,
    y=0,
    size=x_border_size + 0.012,
    angle=0,
))
border_idx += 1
# west border
print(border_template.format(
    border_idx=border_idx,
    x=x_border_size / 2,
    y=-y_border_size,
    size=x_border_size + 0.012,
    angle=0,
))
border_idx += 1
# south border
print(border_template.format(
    border_idx=border_idx,
    x=0,
    y=-y_border_size / 2,
    size=y_border_size + 0.012,
    angle=math.pi / 2,
))
border_idx += 1
# north border
print(border_template.format(
    border_idx=border_idx,
    x=x_border_size,
    y=-y_border_size / 2,
    size=y_border_size + 0.012,
    angle=math.pi / 2,
))

}@
        </model>

        <model name="maze_central_post">
            <static>true</static>
@{
post_template = """
            <link name="post_{post_idx}">
                <pose>{x} {y} 0.025 0 0 0</pose>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.012 0.012 0.05</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <cast_shadows>false</cast_shadows>
                    <geometry>
                        <box>
                            <size>0.012 0.012 0.05</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.9 0.9 1</ambient>
                        <diffuse>0.9 0.9 0.9 1</diffuse>
                    </material>
                </visual>
            </link>
"""
post_idx = 0
post_x = (height // 2) * cell_size
post_y = -(width // 2) * cell_size
print(post_template.format(post_idx=post_idx, x=post_x, y=post_y))
}@
        </model>

        <model name="maze_walls">
            <static>true</static>
@{
wall_idx = 0
wall_template = """
            <link name="wall_{wall_idx}">
                <pose>{x} {y} 0.025 0 0 {angle}</pose>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.192 0.012 0.05</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <cast_shadows>false</cast_shadows>
                    <geometry>
                        <box>
                            <size>0.192 0.012 0.05</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.9 0.9 1</ambient>
                        <diffuse>0.9 0.9 0.9 1</diffuse>
                    </material>
                </visual>
            </link>
"""
# horizontal walls
for row in range(1, height):
    for col in range(width):
        if horizontal_walls[row][col]:
            x = row * cell_size
            y = -col * cell_size - cell_size / 2
            print(wall_template.format(wall_idx=wall_idx, x=x, y=y, angle=math.pi / 2))
            wall_idx += 1

# vertical walls
for row in range(height):
    for col in range(1, width):
        if vertical_walls[row][col]:
            x = row * cell_size + cell_size / 2
            y = -col * cell_size
            print(wall_template.format(wall_idx=wall_idx, x=x, y=y, angle=0))
            wall_idx += 1
}@
        </model>
    </world>
</sdf>
