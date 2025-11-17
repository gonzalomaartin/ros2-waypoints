# ros2-waypoints

ROS2 Waypoints Project (project_rom)

This repository contains a ROS2 package `project_rom` that demonstrates a simple waypoint-following stack for a differential robot (TurtleBot3) with basic obstacle avoidance using sonars. It was developed as part of a robotics project — see the included technical document `Documento técnico ROM.pdf` for full project details.

Key features
- Waypoint publisher that reads a path from `config/path.yaml` and publishes it as a `project_rom/msg/WayPointPath` message.
- A Pure Pursuit controller node that subscribes to odometry and the published path and outputs `cmd_vel` commands.
- An obstacle avoider node that reads sonar sensors and publishes an avoidance curvature used by the controller.
- RViz launch and a CoppeliaSim scene launcher to visualize and simulate the robot in a sonar scene.

Quick start

Requirements
- ROS2 installed and sourced (for example: `source /opt/ros/<distro>/setup.zsh`).
- `colcon` build tools.
- CoppeliaSim installed and available on `PATH` (the `scene_launch.py` executes `coppeliaSim.sh`).
- Typical ROS2 dependencies are declared in `project_rom/package.xml` (e.g. `rclcpp`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2`, `visualization_msgs`).

Build

From the workspace root (this repository):

```bash
# (example) source ROS2
source /opt/ros/<distro>/setup.zsh

# Build
colcon build --packages-select project_rom

# Source install overlay
source install/setup.zsh
```

Running (simple)

You can run the whole scene (CoppeliaSim + RViz + nodes) using the provided launch file:

```bash
ros2 launch project_rom scene_launch.py
```

Notes:
- `scene_launch.py` accepts the `path_file` argument (defaults to `share/project_rom/config/path.yaml`) and `use_sim_time`.
- It launches CoppeliaSim with the scene at `scenes/sonar_scene.ttt`. Ensure `coppeliaSim.sh` is in your `PATH`, or edit the launch to point to your CoppeliaSim executable.
- You can set the robot model via environment variable `TURTLEBOT3_MODEL` (default `burger`), which selects the corresponding URDF in `urdf/`.

Running nodes individually

If you prefer to run nodes separately (for debugging):

```bash
# publish waypoints
ros2 run project_rom waypoints_node --ros-args -p path_file:=$(ros2 pkg prefix project_rom)/share/project_rom/config/path.yaml

# run the controller
ros2 run project_rom pure_pursuit_node

# run the obstacle avoider
ros2 run project_rom obstacle_avoider_node
```

Topics and messages
- `/path` (published by `waypoints_node`) — message type: `project_rom/msg/WayPointPath` (fields: `std_msgs/Bool closed_path`, `geometry_msgs/Point[] points`).
- `/cmd_vel` (published by `pure_pursuit_node`) — message type: `geometry_msgs/Twist`.
- `/odom` (subscribed by `pure_pursuit_node`) — message type: `nav_msgs/Odometry` (must be available in your simulation or robot).
- `/avoidance_curvature` (published by `obstacle_avoider_node`) — message type: `std_msgs/Float64` (signed curvature value used to augment steering).

Message definitions (brief)
- `WayPointPath.msg`:
	- `std_msgs/Bool closed_path` — whether the path is closed (looping).
	- `geometry_msgs/Point[] points` — list of points forming the path.
- `Reference.msg` (present in `msg/`) — contains `std_msgs/Header header`, `geometry_msgs/Pose pose`, `geometry_msgs/Twist velocity`.

Configuration
- `config/path.yaml` contains an example sequence of points used by `waypoints_node` (see file for coordinates). Edit this YAML to change the routes the robot will try to follow.

How it works (short)
- `waypoints_node` reads `config/path.yaml` and publishes the list as a `WayPointPath` message (several times to ensure subscribers receive it).
- `pure_pursuit_node` receives the path and odometry, computes a goal point and steering using the Pure Pursuit algorithm (see `WayPointPathTools` helper for point selection) and publishes velocity commands on `/cmd_vel`.
- `obstacle_avoider_node` listens to sonar topics (e.g. `/sonar0`..`/sonar3`) and publishes a curvature correction on `/avoidance_curvature` that the controller adds to its angular command.

Development notes
- Core sources are under `project_rom/src/`:
	- `waypoints_node.cpp` — reads YAML and publishes path.
	- `pure_pursuit_odom.cpp` — controller using odometry and path.
	- `obstacle_avoider_node.cpp` — sonar-based avoidance and curvature publisher.
	- `waypoint_tools/WayPointPathTools.cpp` — path helper logic used by the controller.
- Build is managed with `ament_cmake` (see `CMakeLists.txt`). The package also auto-generates the `WayPointPath` message from `msg/WayPointPath.msg`.

Tips & troubleshooting
- If the robot doesn't move, check that `/odom` is being published and that the controller parameters (`tolerance`, `Kp`, `linear_speed`) are tuned.
- If `scene_launch.py` fails to start CoppeliaSim, ensure `coppeliaSim.sh` is executable and available in `PATH`, or change `ExecuteProcess` command to the correct path.
- Use `ros2 topic echo /path` to inspect the published waypoints or `ros2 topic echo /avoidance_curvature` to see avoidance outputs.

Reference / Documentation
- Technical project document (Spanish): `Documento técnico ROM.pdf` — open it for design rationale, diagrams and experimental results.

Contributing
- Feel free to open issues or pull requests. If you add sensors or different robots, update `launch/scene_launch.py` and `urdf/` accordingly.

License
- The package currently has no license declared in `package.xml`. Add a LICENSE file and update `package.xml` if you want to open-source this project.

Contact
- For questions about the implementation contact the maintainers listed in `package.xml` or open an issue.

