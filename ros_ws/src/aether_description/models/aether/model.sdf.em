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
wheel_offset = [
    -0.02,
    chassis_size[1] / 2,
    0.0,
]

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

}@
<sdf version="1.8">
    <model name="aether">
        <plugin
            filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
        </plugin>
        <link name="base_link">
            <pose>0 0 0 0 0 0</pose>
            <inertial>
                <mass>0.0</mass>
                <inertia>
                    <ixx>0.0</ixx>
                    <iyy>0.0</iyy>
                    <izz>0.0</izz>
                </inertia>
            </inertial>
        </link>
        <joint name="base_to_chassis" type="fixed">
            <parent>base_link</parent>
            <child>chassis_link</child>
        </joint>

        <link name="chassis_link">
            <pose relative_to="base_link">0 0 0 0 0 0</pose>
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
empy.include(template_path("wheel.sdf.em"), {
    "radius": wheel_radius,
    "length": wheel_length,
    "mass": wheel_mass,
    "side": "left",
    "wheel_offset": wheel_offset,
    "relative_to": "base_link",
    "name": "left_wheel"
})
empy.include(template_path("wheel.sdf.em"), {
    "radius": wheel_radius,
    "length": wheel_length,
    "mass": wheel_mass,
    "side": "right",
    "wheel_offset": wheel_offset,
    "relative_to": "chassis_link",
    "name": "right_wheel"
})
}@
        <link name="lidar_link">
            <pose relative_to="base_link">0 0 0.01 0 0 0</pose>
            <inertial>
                <mass>0.01</mass>
                <inertia>
                    <ixx>0.000166667</ixx>
                    <iyy>0.000166667</iyy>
                    <izz>0.000166667</izz>
                </inertia>
            </inertial>
            <collision name="lidar_collision">
                <geometry>
                    <box>
                        <size>0.01 0.01 0.01</size>
                    </box>
                </geometry>
            </collision>
            <visual name="lidar_visual">
                <geometry>
                    <box>
                        <size>0.01 0.01 0.01</size>
                    </box>
                </geometry>
            </visual>
            <sensor name="gpu_lidar" type="gpu_lidar">
                <pose>0 0 0 0 0 0</pose>
                <topic>scan</topic>
                <gz_frame_id>lidar_link</gz_frame_id>
                <update_rate>10</update_rate>
                <lidar>
                    <scan>
                        <horizontal>
                            <samples>1</samples>
                            <resolution>1</resolution>
                            <min_angle>0.0</min_angle>
                            <max_angle>0.0</max_angle>
                        </horizontal>
                        <vertical>
                            <samples>1</samples>
                            <resolution>1</resolution>
                            <min_angle>0.0</min_angle>
                            <max_angle>0.0</max_angle>
                        </vertical>
                    </scan>
                    <range>
                        <min>0.001</min>
                        <max>1.3</max>
                        <resolution>0.001</resolution>
                    </range>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.01</stddev>
                    </noise>
                </lidar>
                <visualize>true</visualize>
            </sensor>
        </link>

        <joint name="lidar_joint" type="fixed">
            <parent>chassis_link</parent>
            <child>lidar_link</child>
        </joint>

        <plugin
        filename="ignition-gazebo-diff-drive-system"
        name="ignition::gazebo::systems::DiffDrive">
            <left_joint>left_wheel_joint</left_joint>
            <right_joint>right_wheel_joint</right_joint>
            <wheel_separation>0.1</wheel_separation>
            <wheel_radius>0.01</wheel_radius>
            <odom_publish_frequency>5</odom_publish_frequency>
            <max_linear_acceleration>5</max_linear_acceleration>
            <min_linear_acceleration>-5</min_linear_acceleration>
            <max_angular_acceleration>5</max_angular_acceleration>
            <min_angular_acceleration>-5</min_angular_acceleration>
            <max_linear_velocity>3.5</max_linear_velocity>
            <min_linear_velocity>-3.5</min_linear_velocity>
            <max_angular_velocity>5</max_angular_velocity>
            <min_angular_velocity>-5</min_angular_velocity>
        </plugin>
    </model>
</sdf>
