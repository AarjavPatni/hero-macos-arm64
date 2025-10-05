#!/bin/bash

# Gazebo Wizard Launcher
# This script launches the gazebo_wizard.launch file which starts Gazebo with HeRo world + wizard GUI

echo "Starting Gazebo Wizard..."
echo "This will launch Gazebo with the HeRo world and the wizard GUI for robot control"
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

# Check if roscore is running, if not start it
if ! pgrep -x "roscore" > /dev/null; then
    echo "Starting roscore..."
    roscore &
    sleep 3
fi

# Launch the gazebo_wizard.launch file
echo "Launching: roslaunch hero_gazebo gazebo_wizard.launch"
roslaunch hero_gazebo gazebo_wizard.launch

echo ""
echo "gazebo_wizard.launch has finished."
read -p "Press any key to close this window..."