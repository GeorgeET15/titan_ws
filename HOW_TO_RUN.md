# How to Run Titan V1

## Prerequisites
Ensure you have the following installed:
- ROS 2 (Jazzy or Humble)
- Gazebo Sim (Harmonic or compatible)
- `ros_gz` packages

## Build the Workspace
Navigate to your workspace root and build the package:

```bash
cd ~/titan_ws
colcon build --symlink-install
```

## Run the Simulation
Source the workspace and launch the simulation:

```bash
source install/setup.bash
ros2 launch titan_v1 titan_v1.launch.py
```

## Control the Robot
The launch file attempts to open a separate terminal for keyboard control. If it does not appear, open a new terminal and run:

```bash
source ~/titan_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys shown on the screen (usually `i`, `j`, `k`, `l`, `,`) to move the robot.
