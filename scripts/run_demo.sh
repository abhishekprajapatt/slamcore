#!/bin/bash

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Workspace not built. Run ./scripts/build.sh first"
    exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"

echo "Starting Slamcore demo..."
echo ""
echo "Launching Gazebo + ROS2 SLAM system..."
echo ""

ros2 launch slamcore gazebo_slam.launch.py gui:=true &
GAZEBO_PID=$!

sleep 3

echo "Launching RViz2 visualization..."
ros2 launch slamcore rviz.launch.py &
RVIZ_PID=$!

echo ""
echo "Slamcore demo is running!"
echo ""
echo "To simulate robot motion, publish to /cmd_vel:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'"
echo ""
echo "To stop the demo, press Ctrl+C or close Gazebo/RViz windows"
echo ""

wait $GAZEBO_PID
