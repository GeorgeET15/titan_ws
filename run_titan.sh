#!/bin/bash

################################################################################
# TITAN V1 — FULL BUILD + RUN SCRIPT
# This script:
#   1. Sources ROS 2 + Gazebo Sim
#   2. Builds the entire workspace (colcon build)
#   3. Sources install/
#   4. Launches titan_v1.launch.py
#
# Usage:
#   chmod +x run_titan.sh
#   ./run_titan.sh
################################################################################

set -e   # Exit on error

echo "-------------------------------------------------------"
echo "  TITAN V1: ROS 2 Build + Launch Script"
echo "-------------------------------------------------------"

################################################################################
# 1. Detect workspace root
################################################################################

# If script is inside the workspace, move to workspace root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Assume the script is inside <workspace>/ (same level as src/)
# If src/ does NOT exist, exit with error
if [ ! -d "$SCRIPT_DIR/src" ]; then
    echo "ERROR: No src/ folder found in: $SCRIPT_DIR"
    echo "Place this script in your ROS workspace (same level as src/)."
    exit 1
fi

cd "$SCRIPT_DIR"

echo "Workspace root: $SCRIPT_DIR"
echo ""

################################################################################
# 2. Source ROS 2 & Gazebo Sim ENV
# Update to correct distro if required (humble/iron/jazzy etc.)
################################################################################

# Detect ROS installation
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: Could not find ROS 2 installation!"
    exit 1
fi

echo "ROS 2 environment sourced."
echo ""

# Source Gazebo Sim (Fortress / Garden / Harmonic)
if [ -f "/usr/share/gz/gz-harmonic/setup.sh" ]; then
    source /usr/share/gz/gz-harmonic/setup.sh
elif [ -f "/usr/share/gz/gz-garden/setup.sh" ]; then
    source /usr/share/gz/gz-garden/setup.sh
elif [ -f "/usr/share/gz/gz-fortress/setup.sh" ]; then
    source /usr/share/gz/gz-fortress/setup.sh
else
    echo "WARNING: Could not detect Gazebo Sim setup."
fi

echo "Gazebo Sim environment sourced (if available)."
echo ""

################################################################################
# 3. Build the workspace
################################################################################

echo "-------------------------------------------------------"
echo "  Building workspace with colcon..."
echo "-------------------------------------------------------"

colcon build --symlink-install --event-handlers console_direct+

echo ""
echo "Build completed successfully."
echo ""

################################################################################
# 4. Source the install folder
################################################################################

source install/setup.bash
echo "Workspace environment sourced."
echo ""

################################################################################
# 5. Launch the robot
################################################################################

echo "-------------------------------------------------------"
echo "  Launching TITAN V1 robot in Gazebo..."
echo "-------------------------------------------------------"
echo ""

ros2 launch titan_v1 titan_v1.launch.py
