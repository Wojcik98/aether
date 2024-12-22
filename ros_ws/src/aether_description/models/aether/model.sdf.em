<?xml version="1.0" ?>
@{
# inputs: config
import math
import os
from ament_index_python.packages import get_package_share_directory

from aether_description.sdf_utils import (
    empty_inertial,
    material,
    box_inertial,
    pose_2D_to_3D,
)

chassis_size = config["chassis_size"]
chassis_mass = config["chassis_mass"]

wheel_radius = config["wheel_radius"]
wheel_length = config["wheel_length"]
wheel_mass = config["wheel_mass"]

motor_radius = config["motor_radius"]
motor_length = config["motor_length"]
motor_mass = config["motor_mass"]

tofs_poses = config["tofs_poses"]

# Calculated properties
wheel_offset_x = wheel_radius * 1.1
wheel_offset_y = chassis_size[1] / 2
wheel_offset_z = wheel_radius * 0.6

pkg = os.path.join(get_package_share_directory("aether_description"))

def template_path(file_path):
    return os.path.join(pkg, "models", "aether", file_path)

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
            <pose relative_to="base_link">
                @(config["base_link_to_chassis"][0])
                @(config["base_link_to_chassis"][1])
                0
                0
                0
                @(config["base_link_to_chassis"][2])
            </pose>
            @(box_inertial(chassis_size, chassis_mass))
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
for tof_name, tof_pose in tofs_poses.items():
    empy.include(template_path("tof.sdf.em"), {
        "name": f"tof_{tof_name}",
        "pose": pose_2D_to_3D(tof_pose),
        "relative_to": "base_link",
        "freq" : config["freq_tofs"]
    })

# IMU
empy.include(template_path("imu.sdf.em"), {
    "name": "imu",
    "pose": [0, 0, chassis_size[2] / 2, 0, 0, 0],
    "relative_to": "base_link",
    "freq": config["freq_imu_enc"]
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
            <wheel_separation>@(config["wheel_base"])</wheel_separation>
            <wheel_radius>@(wheel_radius)</wheel_radius>
            <min_acceleration>-1.0</min_acceleration>
            <max_acceleration>1.0</max_acceleration>
            <odom_publish_frequency>@(config["freq_imu_enc"])</odom_publish_frequency>
            <frame_id>odom</frame_id>
            <child_frame_id>base_link_enc</child_frame_id>
        </plugin>
    </model>
</sdf>
