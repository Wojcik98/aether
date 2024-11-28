@{
# inputs: mass, radius, length
xy = mass * (3 * radius * radius + length * length) / 12.0
z = mass * radius * radius * 0.5
}@
<inertial>
  <mass>@(mass)</mass>
  <inertia>
    <ixx>@(xy)</ixx>
    <ixy>0.0</ixy>
    <ixz>0.0</ixz>
    <iyy>@(xy)</iyy>
    <iyz>0.0</iyz>
    <izz>@(z)</izz>
  </inertia>
</inertial>
