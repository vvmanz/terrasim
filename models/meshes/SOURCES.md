# Robot meshes: sources and licences

The meshes are visuals only: wheels and contact patches are computed by the plugin from the parameters in
`tools/robots.py`. Each mesh is placed with the pose of the original URDF / xacro, moved to the terrasim model frame
(origin on the ground below the robot, x forward). Previews with the physical wheel radius drawn in red:
`python3 tools/robot_preview.py` → `docs/robots/`.

These files keep their own licences (listed below); they are not covered by the MIT licence of terrasim.

| folder | source | licence | changes |
|---|---|---|---|
| `husky/` | husky_description (Clearpath Robotics): base_link, top_chassis, user_rail, bumper, wheel | BSD-3-Clause (`husky/LICENSE.txt`) | none; the wheel is scaled by 0.1651/0.1778 at load time so that its radius matches the physical one |
| `turtlebot3_burger/` | turtlebot3_description (ROBOTIS): burger_base, left/right_tire, lds | Apache 2.0 (`LICENSE.txt`) | **modified**: decimated by vertex clustering with a 1 mm grid (`tools/meshes.py`), 96k → 52k triangles |
| `turtlebot3_waffle_pi/` | turtlebot3_description (ROBOTIS): waffle_base, left/right_tire (lds from burger) | Apache 2.0 (`LICENSE.txt`) | **modified**: the base decimated with a 1.6 mm grid, 326k → 83k triangles |
| `jackal/` | jackal_description (Clearpath Robotics): jackal-base, jackal-fender, jackal-wheel | BSD-3-Clause (`LICENSE.txt`) | none; the wheel is scaled by 0.098/0.100 at load time |
| `pioneer3dx/` | p3dx_description (J. Horner): chassis, top, swivel, center_wheel, left/right_wheel | MIT (`LICENSE.txt`) | none; wheels scaled at load time to the 195 mm datasheet diameter, the caster to r = 35 mm |
| `youbot/` | youbot_description (KUKA youBot / Locomotec): base, plate, mecanum_left/right, arm0–5, palm, finger | GPLv3 (`youbot/NOTICE.txt`) | none; the arm in the folded rest pose (all joint angles 0) |

Robotino and the MarsSim rover are procedural (no meshes).
