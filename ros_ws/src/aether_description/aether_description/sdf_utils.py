#!/usr/bin/env python3
from dataclasses import dataclass


@dataclass
class Maze:
    width: int
    height: int
    horizontal_walls: list[list[bool]]
    vertical_walls: list[list[bool]]


def decode_maze_from_text(path: str) -> Maze:
    """Decodes a maze from a text file."""
    with open(path) as f:
        lines = f.readlines()
    top_height = len(lines)
    height = (len(lines) - 1) // 2
    width = (len(lines[0].strip()) - 1) // 4

    horizontal_walls = [
        [lines[2 * row][4 * col + 1] == "-" for col in range(width)]
        for row in range(height, -1, -1)
    ]
    vertical_walls = [
        [lines[2 * row - 1][4 * col] == "|" for col in range(width + 1)]
        for row in range(height, 0, -1)
    ]
    return Maze(width, height, horizontal_walls, vertical_walls)


def pose_2D_to_3D(pose2D):
    """Converts a 2D pose to a 3D pose."""
    return [pose2D[0], pose2D[1], 0, 0, 0, pose2D[2]]


def empty_inertial():
    """Prints an inertial block with tiny mass and inertia."""
    print(
        """
        <inertial>
            <mass>0.0000001</mass>
            <inertia>
                <ixx>0.0000001</ixx>
                <ixy>0.0</ixy>
                <ixz>0.0</ixz>
                <iyy>0.0000001</iyy>
                <iyz>0.0</iyz>
                <izz>0.0000001</izz>
            </inertia>
        </inertial>
    """
    )


_colors = {
    "grey": "0.8 0.8 0.8 1",
    "black": "0 0 0 1",
    "purple": "0.5 0 0.5 1",
}


def material(color_name):
    """Prints a material with a given color."""
    color = _colors.get(color_name, "0.8 0.8 0.8 1")
    print(
        f"""
        <material>
          <diffuse>{color}</diffuse>
          <specular>{color}</specular>
          <ambient>{color}</ambient>
        </material>
    """
    )


def box_geometry(size):
    """Prints a box geometry with a given size."""
    print(
        f"""
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
    """
    )


def box_inertial(size, mass):
    """Prints an inertial block for a box with a given size and mass."""
    x, y, z = size
    ixx = (1 / 12) * mass * (y**2 + z**2)
    iyy = (1 / 12) * mass * (x**2 + z**2)
    izz = (1 / 12) * mass * (x**2 + y**2)
    print(
        f"""
        <inertial>
            <mass>{mass}</mass>
            <inertia>
                <ixx>{ixx}</ixx>
                <ixy>0.0</ixy>
                <ixz>0.0</ixz>
                <iyy>{iyy}</iyy>
                <iyz>0.0</iyz>
                <izz>{izz}</izz>
            </inertia>
        </inertial>
    """
    )


def cylinder_geometry(radius, length):
    """Prints a cylinder geometry with a given radius and length."""
    print(
        f"""
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
    """
    )


def cylinder_inertial(radius, length, mass):
    """Prints an inertial block for a cylinder with a given radius, length, and mass."""
    ixx = (1 / 12) * mass * (3 * radius**2 + length**2)
    iyy = ixx
    izz = (1 / 2) * mass * radius**2
    print(
        f"""
        <inertial>
            <mass>{mass}</mass>
            <inertia>
                <ixx>{ixx}</ixx>
                <ixy>0.0</ixy>
                <ixz>0.0</ixz>
                <iyy>{iyy}</iyy>
                <iyz>0.0</iyz>
                <izz>{izz}</izz>
            </inertia>
        </inertial>
    """
    )
