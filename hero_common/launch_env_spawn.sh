#!/bin/bash

# HeRo Environment Spawn Launcher
# This script launches the env_spawn.launch file which spawns arena_1 and 4 HeRo robots

echo "Starting HeRo Environment Spawn..."
echo "This will spawn arena_1 and 4 HeRo robots in Gazebo"
echo "Make sure Gazebo is already running with the HeRo world!"
echo ""

# Ensure we're in the right directory
cd /catkin_ws

# Source ROS environment with error checking
echo "Sourcing ROS environment..."
if ! source /opt/ros/noetic/setup.bash; then
    echo "ERROR: Failed to source ROS environment"
    read -p "Press any key to close..."
    exit 1
fi

# Build workspace if not built yet or if build is incomplete
if [ ! -f "/catkin_ws/devel/setup.bash" ] || [ ! -d "/catkin_ws/devel/share/hero_gazebo" ]; then
    echo "Building catkin workspace..."
    rm -rf build devel logs
    if ! catkin build; then
        echo "ERROR: Failed to build workspace"
        read -p "Press any key to close..."
        exit 1
    fi
    echo "Workspace built successfully!"
fi

echo "Sourcing workspace environment..."
if ! source /catkin_ws/devel/setup.bash; then
    echo "ERROR: Failed to source workspace environment"
    read -p "Press any key to close..."
    exit 1
fi

# Set ROS networking (fix ROS_IP issues)
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP

# Check if roscore is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "Warning: roscore doesn't seem to be running!"
    echo "Make sure to start roscore or the gazebo_wizard first"
    echo ""
fi

# Launch the env_spawn.launch file
echo "Launching: roslaunch hero_gazebo env_spawn.launch"
roslaunch hero_gazebo env_spawn.launch

echo ""
echo "env_spawn.launch has finished."
read -p "Press any key to close this window..."