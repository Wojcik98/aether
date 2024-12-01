<?xml version="1.0" ?>
@{
import math
import os
from ament_index_python.packages import get_package_share_directory

# Inputs
chassis_size = [0.08, 0.08, 0.002]
chassis_mass = 0.03

wheel_radius = 0.01
wheel_length = 0.01
wheel_mass = 0.010

# Calculated properties
wheel_offset_x = wheel_radius * 1.1
wheel_offset_y = chassis_size[1] / 2 + 0.001
wheel_offset_z = wheel_radius * 0.6

motor_radius = 0.008
motor_length = 0.02
motor_mass = 0.02

pkg = os.path.join(get_package_share_directory("aether_description"))

def template_path(file_path):
    return os.path.join(pkg, "models", "aether", file_path)

colors = {
    "grey": "0.8 0.8 0.8 1",
    "black": "0 0 0 1",
    "purple": "0.5 0 0.5 1",
}

def material(color_name):
    """Prints a material with a given color"""
    color = colors.get(color_name, "0.8 0.8 0.8 1")
    print(f"""
        <material>
          <diffuse>{color}</diffuse>
          <specular>{color}</specular>
          <ambient>{color}</ambient>
        </material>
    """)

def empty_inertial():
    """Prints an empty inertial block"""
    print("""
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
    """)

}@
<sdf version="1.8">
    <model name="aether">
        <plugin
            filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
        </plugin>
        <link name="base_link">
            <pose>0 0 0 0 0 0</pose>
            @(empty_inertial())
        </link>
        <joint name="base_to_chassis" type="fixed">
            <parent>base_link</parent>
            <child>chassis_link</child>
        </joint>

        <link name="chassis_link">
            <pose relative_to="base_link">0.01 0 0 0 0 0</pose>
@{
empy.include(template_path("inertial_box.sdf.em"), {
    "size": chassis_size,
    "mass": chassis_mass
})
}@
            <visual name="chassis_visual">
                <geometry>
                    <box>
                        <size>
                            @(chassis_size[0])
                            @(chassis_size[1])
                            @(chassis_size[2])
                        </size>
                    </box>
                </geometry>
                @(material("purple"))
            </visual>
            <collision name="chassis_collision">
                <geometry>
                    <box>
                        <size>
                            @(chassis_size[0])
                            @(chassis_size[1])
                            @(chassis_size[2])
                        </size>
                    </box>
                </geometry>
            </collision>
        </link>

@{
# ToF sensors
empy.include(template_path("tof.sdf.em"), {
    "name": "tof_right_side",
    "pose": [0.02, -0.035, 0, 0, 0, -math.pi / 2],
    "relative_to": "chassis_link"
})
empy.include(template_path("tof.sdf.em"), {
    "name": "tof_right_diag",
    "pose": [0.03, 0.015, 0, 0, 0, -math.pi / 4],
    "relative_to": "chassis_link"
})
empy.include(template_path("tof.sdf.em"), {
    "name": "tof_right_front",
    "pose": [0.03, -0.025, 0, 0, 0, 0],
    "relative_to": "chassis_link"
})
empy.include(template_path("tof.sdf.em"), {
    "name": "tof_left_front",
    "pose": [0.03, 0.025, 0, 0, 0, 0],
    "relative_to": "chassis_link"
})
empy.include(template_path("tof.sdf.em"), {
    "name": "tof_left_diag",
    "pose": [0.03, -0.015, 0, 0, 0, math.pi / 4],
    "relative_to": "chassis_link"
})
empy.include(template_path("tof.sdf.em"), {
    "name": "tof_left_side",
    "pose": [0.02, 0.035, 0, 0, 0, math.pi / 2],
    "relative_to": "chassis_link"
})

# Wheels
empy.include(template_path("wheel.sdf.em"), {
    "radius": wheel_radius,
    "length": wheel_length,
    "mass": wheel_mass,
    "side": "left",
    "wheel_offset": [wheel_offset_x, wheel_offset_y, wheel_offset_z],
    "relative_to": "base_link",
    "name": "left_front_wheel"
})
empy.include(template_path("wheel.sdf.em"), {
    "radius": wheel_radius,
    "length": wheel_length,
    "mass": wheel_mass,
    "side": "left",
    "wheel_offset": [-wheel_offset_x, wheel_offset_y, wheel_offset_z],
    "relative_to": "base_link",
    "name": "left_rear_wheel"
})
empy.include(template_path("wheel.sdf.em"), {
    "radius": wheel_radius,
    "length": wheel_length,
    "mass": wheel_mass,
    "side": "right",
    "wheel_offset": [wheel_offset_x, -wheel_offset_y, wheel_offset_z],
    "relative_to": "base_link",
    "name": "right_front_wheel"
})
empy.include(template_path("wheel.sdf.em"), {
    "radius": wheel_radius,
    "length": wheel_length,
    "mass": wheel_mass,
    "side": "right",
    "wheel_offset": [-wheel_offset_x, -wheel_offset_y, wheel_offset_z],
    "relative_to": "base_link",
    "name": "right_rear_wheel"
})

# Motors
empy.include(template_path("motor.sdf.em"), {
    "name": "left_motor",
    "relative_to": "base_link",
    "pose": [0, motor_length * 0.7, motor_radius + chassis_size[2] / 2, -math.pi / 2, 0, 0],
    "motor_radius": motor_radius,
    "motor_length": motor_length,
    "motor_mass": motor_mass
})
empy.include(template_path("motor.sdf.em"), {
    "name": "right_motor",
    "relative_to": "base_link",
    "pose": [0, -motor_length * 0.7, motor_radius + chassis_size[2] / 2, math.pi / 2, 0, 0],
    "motor_radius": motor_radius,
    "motor_length": motor_length,
    "motor_mass": motor_mass
})
}@

        <plugin
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
            <left_joint>left_front_wheel_joint</left_joint>
            <left_joint>left_rear_wheel_joint</left_joint>
            <right_joint>right_front_wheel_joint</right_joint>
            <right_joint>right_rear_wheel_joint</right_joint>
            <wheel_separation>@(chassis_size[1] + wheel_length )</wheel_separation>
            <wheel_radius>@(wheel_radius)</wheel_radius>
            <odom_publish_frequency>5</odom_publish_frequency>
            <max_linear_acceleration>5</max_linear_acceleration>
            <min_linear_acceleration>-5</min_linear_acceleration>
            <max_angular_acceleration>20</max_angular_acceleration>
            <min_angular_acceleration>-20</min_angular_acceleration>
            <max_linear_velocity>3.5</max_linear_velocity>
            <min_linear_velocity>-3.5</min_linear_velocity>
            <max_angular_velocity>10</max_angular_velocity>
            <min_angular_velocity>-10</min_angular_velocity>
        </plugin>
    </model>
</sdf>
