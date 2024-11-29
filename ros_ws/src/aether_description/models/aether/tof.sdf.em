@{
# inputs: name, pose, relative_to
tof_size = [0.003, 0.01, 0.01]
}@
<joint name="@(name)_joint" type="fixed">
    <parent>@(relative_to)</parent>
    <child>@(name)_link</child>
</joint>
<link name="@(name)_link">
    <pose relative_to="@(relative_to)">
        @(pose[0])
        @(pose[1])
        @(pose[2] + tof_size[2] / 2)
        @(pose[3])
        @(pose[4])
        @(pose[5])
    </pose>
@{
empy.include(template_path("inertial_box.sdf.em"), {
    "size": tof_size,
    "mass": 0.003
})
}@
    <!-- the sensor doesn't need collision -->
    <visual name="@(name)_visual">
        <geometry>
            <box>
                <size>@(tof_size[0]) @(tof_size[1]) @(tof_size[2])</size>
            </box>
        </geometry>
        @(material("purple"))
    </visual>
</link>
<joint name="@(name)_optical_frame_joint" type="fixed">
    <parent>@(name)_link</parent>
    <child>@(name)_optical_frame</child>
</joint>
<link name="@(name)_optical_frame">
    <pose relative_to="@(name)_link">@(tof_size[0] / 2) 0 0 0 0 0</pose>
    @(empty_inertial())
    <sensor name="gpu_lidar" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose>
        <topic>@(name)/scan</topic>
        <gz_frame_id>@(name)_optical_frame</gz_frame_id>
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
                <stddev>0.002</stddev>
            </noise>
        </lidar>
        <visualize>true</visualize>
    </sensor>
</link>
