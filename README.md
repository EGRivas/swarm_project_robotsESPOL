# Swarm Robot Simulation in ROS2 Jazzy & Gazebo Harmonic
By Emmanuel Rivas, Genesis Vargas and Eliezer Acebo. 
Mechatronics Engineering students from Escuela Superior Politécnica del Litoral (ESPOL) | Polytechnic Higher School of the Coast.
A comprehensive swarm robotics simulation implementing flocking behavior for aquatic surface robots using ROS2 Jazzy and Gazebo Harmonic.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)  
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Data Collection](#data-collection)
- [Troubleshooting](#troubleshooting)

## Features

- **Swarm Flocking Behavior**: Implementation of separation, alignment, and cohesion rules
- **Aquatic Surface Navigation**: Specialized for water surface robot simulation
- **Real-time Data Logging**: CSV export of velocities, positions, and trajectories
- **Multi-robot Coordination**: Support for a maximum of 7 robots in swarm formation (maximum stable work at 5 robots)
- **Gazebo Harmonic Integration**: Physics simulation with Gazebo Harmonic
- **Obstacle Avoidance**: Laser-based collision avoidance (optional)
- **URDF Visualization**: RViz integration with joint state publisher

## Prerequisites

Before installing, ensure you have:

- **Ubuntu 24.04 LTS (Noble Nobis)**
- **ROS2 Jazzy Jalisco**
- **Gazebo Harmonic**
- **Python 3.12+**

### System Dependencies

```bash
# ROS2 Jazzy installation
sudo apt update && sudo apt install curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Gazebo Harmonic installation
sudo apt install gz-harmonic

# Additional ROS2 packages
sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-rviz2
```

## Installation
1. Clone the Repository
```bash
# Create workspace
mkdir -p ~/swarm_project/src
cd ~/swarm_project/src

# Clone repository
git clone https://github.com/your-username/swarm_project.git .
```
2. Install Dependencies
```bash
cd ~/swarm_project

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
3. Build the Project
```bash
# Build all packages
colcon build --symlink-install

# Source the workspace
echo "source ~/swarm_project/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4. Verify Installation
```bash
# Check if packages are properly installed
ros2 pkg list | grep swarm

# Expected output:
# swarm_bringup
# swarm_control  
# swarm_robot_description
# swarm_worlds
```
5. Environment Setup
```bash
# Verify Gazebo Harmonic
gz sim --version

# Verify ROS2 Jazzy
ros2 --version

# Test workspace sourcing
ros2 launch --help
```

##Project Structure
The project consists of 4 main packages:

-swarm_robot_description: Robot URDF/XACRO definitions and meshes
-swarm_worlds: Gazebo world files and environments
-swarm_control: Flocking algorithms and robot controllers
-swarm_bringup: Launch files for simulation orchestration

###Build Verification
After installation, verify each package builds correctly:
```bash
# Test individual packages
colcon build --packages-select swarm_robot_description
colcon build --packages-select swarm_worlds
colcon build --packages-select swarm_control
colcon build --packages-select swarm_bringup
```

###Quick Test
Test your installation with a single robot:
```bash
# Terminal 1: Launch test environment
ros2 launch swarm_bringup bringup_gz.launch.py

# You should see:
# - Gazebo Harmonic window opening
# - A single robot spawning in flat ocean world
# - Robot moving with smooth aquatic navigation
```

If the test runs successfully, your installation is complete.

