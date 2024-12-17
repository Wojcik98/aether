@{
# inputs: name, pose, relative_to, freq
from aether_description.sdf_utils import empty_inertial, material, box_inertial, box_geometry
imu_size = [0.001, 0.001, 0.001]
imu_mass = 0.0005
}@
<joint name="@(name)_joint" type="fixed">
    <parent>@(relative_to)</parent>
    <child>@(name)_link</child>
</joint>
<link name="@(name)_link">
    <pose relative_to="@(relative_to)">
        @(pose[0])
        @(pose[1])
        @(pose[2] + imu_size[2] / 2)
        @(pose[3])
        @(pose[4])
        @(pose[5])
    </pose>
    @(box_inertial(imu_size, imu_mass))
    <!-- the sensor doesn't need collision -->
    <visual name="@(name)_visual">
        @(box_geometry(imu_size))
        @(material("black"))
    </visual>
    <sensor name="imu" type="imu">
        <pose>0 0 0 0 0 0</pose>
        <topic>@(name)</topic>
        <update_rate>@(freq)</update_rate>
        <imu>
            <enable_orientation>false</enable_orientation>
        </imu>
    </sensor>
</link>
