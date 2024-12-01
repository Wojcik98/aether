@{
# inputs: radius, length, mass, side, wheel_offset[3], relative_to, name
from aether_description.sdf_utils import material, cylinder_inertial, cylinder_geometry
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
        @(cylinder_geometry(radius, length))
        @(material("black"))
    </visual>
    <collision name="collision">
        @(cylinder_geometry(radius, length))
    </collision>
    @(cylinder_inertial(radius, length, mass))
</link>
