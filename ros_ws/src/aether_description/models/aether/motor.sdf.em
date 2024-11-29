@{
# inputs: name, relative_to, pose, motor_radius, motor_length, motor_mass
}@
<link name="@(name)_link">
    <pose relative_to="@(relative_to)">
        @(pose[0])
        @(pose[1])
        @(pose[2])
        @(pose[3])
        @(pose[4])
        @(pose[5])
    </pose>
@{
empy.include(template_path("inertial_cylinder.sdf.em"), {
    "mass": motor_mass,
    "radius": motor_radius,
    "length": motor_length,
})
}@
    <visual name="@(name)_visual">
        <geometry>
            <cylinder>
                <radius>@(motor_radius)</radius>
                <length>@(motor_length)</length>
            </cylinder>
        </geometry>
        @(material("black"))
    </visual>
</link>
<joint name="@(name)_joint" type="fixed">
    <parent>@(relative_to)</parent>
    <child>@(name)_link</child>
</joint>
