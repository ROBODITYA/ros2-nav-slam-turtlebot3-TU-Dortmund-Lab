# ROS2 Autonomous Navigation & SLAM Pipeline (TurtleBot3)

Autonomous navigation and SLAM stack built in **ROS2 (Humble)** and **Gazebo** using **TurtleBot3**, with MATLAB used as an external command/control client for global path planning. Developed as part of coursework at TU Dortmund (Nov 2025).

## Overview

This project implements a complete navigation pipeline — from raw sensor input to a navigable map and closed-loop autonomous driving:

- **Nav2 action clients** for sending navigation goals and monitoring feedback/results
- **FSM-based obstacle avoidance** layered on top of Nav2 for reactive behavior
- **TF2 frame transforms** for real-time localization across `map` → `odom` → `base_link` → sensor frames
- **Point cloud mapping** and **NDT (Normal Distributions Transform) scan matching** for 3D environment reconstruction
- **MATLAB integration** — global path planning (RRT/PRM) and a TEB local planner, with MATLAB issuing high-level navigation commands to the ROS2 stack via the ROS Toolbox
- Custom **ROS2 publisher/subscriber nodes** and **action server interfaces**
- Build management with **CMake** and **colcon** across multiple ROS2 packages
- Validated in **Gazebo simulation**, with configuration for deployment on the physical TurtleBot3

## Architecture

```
MATLAB (ROS Toolbox)
   │  global planner (RRT/PRM) + high-level goals
   ▼
ROS2 Action Client  ──►  Nav2 Stack (planner/controller/BT navigator)
   │                          │
   ▼                          ▼
FSM Obstacle Avoidance   TEB Local Planner
   │                          │
   ▼                          ▼
TF2 Transform Tree  ◄──  Sensor Drivers (LiDAR / camera)
   │
   ▼
Point Cloud Mapping + NDT Scan Matching ──► Occupancy/3D Map
```

## Repository Structure

```
.
├── src/
│   ├── <planner_pkg>/          # FSM obstacle avoidance, action clients
│   ├── <mapping_pkg>/          # Point cloud mapping, NDT scan matching
│   ├── <bringup_pkg>/          # Launch files, Nav2 + TEB config, TF setup
│   └── <interfaces_pkg>/       # Custom action/msg/srv definitions
├── matlab/
│   └── ...                     # MATLAB scripts (RRT/PRM planner, ROS2 node)
├── config/
│   └── ...                     # Nav2 params, TEB local planner params, costmap configs
├── worlds/
│   └── ...                     # Gazebo world files
└── README.md
```

> Replace the `<package_name>` placeholders above with your actual package names before publishing.

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Gazebo (classic, matching your ROS2 Humble install)
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_simulations`, `turtlebot3_msgs`)
- Nav2 (`navigation2`, `nav2_bringup`)
- MATLAB with ROS Toolbox (tested with MATLAB R2023b or later)
- colcon (`python3-colcon-common-extensions`)

```bash
export TURTLEBOT3_MODEL=burger
sudo apt install ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup
```

## Build

```bash
# Clone into your workspace src folder
cd ~/ros2_ws/src
git clone <your-repo-url>.git

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source
source install/setup.bash
```

## Running the Simulation

**1. Launch Gazebo with TurtleBot3**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo <your_world>.launch.py
```

**2. Launch Nav2 + TEB local planner + TF setup**
```bash
ros2 launch <bringup_pkg> navigation.launch.py use_sim_time:=true
```

**3. Launch the FSM obstacle avoidance / action client node**
```bash
ros2 run <planner_pkg> fsm_obstacle_avoidance_node
```

**4. Launch point cloud mapping + NDT scan matching**
```bash
ros2 launch <mapping_pkg> ndt_mapping.launch.py
```

**5. Command the robot from MATLAB**
```matlab
% From the matlab/ directory
rosinit  % or ros2node for ROS2, depending on your toolbox setup
run_rrt_planner   % computes global path via RRT/PRM
send_nav2_goal    % sends the goal to the ROS2 action server
```

## Validation

- Verified TF tree consistency (`ros2 run tf2_tools view_frames`) across `map`, `odom`, and sensor frames
- Verified costmap generation and `use_sim_time` consistency across all `ros__parameters` blocks
- Tested obstacle avoidance FSM against static and dynamic obstacles in Gazebo
- Cross-validated NDT scan matching output against ground-truth poses in simulation
- Configuration paths included for porting to the physical TurtleBot3

## Notes

- If costmaps fail to populate, check that `use_sim_time` is set consistently to `true` (simulation) or `false` (hardware) across **all** nodes and launch files — a mismatch here is a common failure mode.
- MATLAB communicates with the ROS2 graph via the ROS Toolbox; ensure `ROS_DOMAIN_ID` matches between your ROS2 shell and MATLAB session.

## License

Add a license (e.g. MIT) if you intend this to be reused by others.
