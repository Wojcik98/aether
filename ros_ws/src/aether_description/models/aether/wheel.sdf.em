@{
# inputs: radius, length, mass, side, wheel_offset[3], relative_to, name
import math
y_offset = wheel_offset[1]
if side == "right":
    y_offset -= length / 2
else:
    y_offset += length / 2
roll = -math.pi / 2 if side == "left" else math.pi / 2
z_axis = "0 0 1" if side == "left" else "0 0 -1"
}@
<joint name="@(name)_joint" type="revolute">
  <parent>@(relative_to)</parent>
  <child>@(name)</child>
  <axis>
    <xyz>@(z_axis)</xyz>
  </axis>
</joint>
<link name="@(name)">
    <pose relative_to="@(relative_to)">
        @(wheel_offset[0])
        @(y_offset)
        @(wheel_offset[2])
        @(roll)
        0
        0
    </pose>
    <visual name="visual">
        <geometry>
            <cylinder>
                <radius>@(radius)</radius>
                <length>@(length)</length>
            </cylinder>
        </geometry>
        @(material("black"))
    </visual>
    <collision name="collision">
        <geometry>
            <cylinder>
                <radius>@(radius)</radius>
                <length>@(length)</length>
            </cylinder>
        </geometry>
    </collision>
@{
empy.include(template_path("inertial_cylinder.sdf.em"), {
    "mass": wheel_mass,
    "radius": radius,
    "length": length,
})
}@
</link>
