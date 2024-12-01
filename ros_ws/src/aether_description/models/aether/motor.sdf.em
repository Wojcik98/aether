@{
# inputs: name, relative_to, pose, motor_radius, motor_length, motor_mass
from aether_description.sdf_utils import material, cylinder_inertial, cylinder_geometry
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
    @(cylinder_inertial(motor_radius, motor_length, motor_mass))
    <visual name="@(name)_visual">
        @(cylinder_geometry(motor_radius, motor_length))
        @(material("black"))
    </visual>
</link>
<joint name="@(name)_joint" type="fixed">
    <parent>@(relative_to)</parent>
    <child>@(name)_link</child>
</joint>
