# Swarm Robot Simulation in ROS2 Jazzy & Gazebo Harmonic
By Emmanuel Rivas, Genesis Vargas and Eliezer Acebo. 
Mechatronics Engineering students from Escuela Superior Politécnica del Litoral (ESPOL) | Polytechnic Higher School of the Coast.
A comprehensive swarm robotics simulation implementing flocking behavior for aquatic surface robots using ROS2 Jazzy and Gazebo Harmonic.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)  
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Data Collection](#data-collection)
- [Troubleshooting](#troubleshooting)

## Features

- **Swarm Flocking Behavior**: Implementation of separation, alignment, and cohesion rules.
- **Aquatic Surface Navigation**: Specialized for water surface robot simulation.
- **Real-time Data Logging**: CSV export of velocities, positions, and trajectories.
- **Multi-robot Coordination**: Support for a maximum of 7 robots in swarm formation (maximum stable work at 5 robots).
- **Gazebo Harmonic Integration**: Physics simulation with Gazebo Harmonic.
- **Obstacle Avoidance**: Laser-based collision avoidance (optional).
- **URDF Visualization**: RViz integration with joint state publisher.

## Prerequisites

Before installing, ensure you have:

- **Ubuntu 24.04 LTS (Noble Nobis).**
- **ROS2 Jazzy Jalisco.**
- **Gazebo Harmonic.**
- **Python 3.12+.**

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
git clone https://github.com/EGRivas/swarm_project.git .
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

## Project Structure
The project consists of 4 main packages:

- **swarm_robot_description:** Robot URDF/XACRO definitions and meshes.
- **swarm_worlds:** Gazebo world files and environments.
- **swarm_control:** Flocking algorithms and robot controllers.
- **swarm_bringup:** Launch files for simulation orchestration.

### Build Verification
After installation, verify each package builds correctly:
```bash
# Test individual packages
colcon build --packages-select swarm_robot_description
colcon build --packages-select swarm_worlds
colcon build --packages-select swarm_control
colcon build --packages-select swarm_bringup
```

### Quick Test
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


## Usage

The swarm simulation consists of multiple launch files that must be executed in sequence. There are two main operational modes: **production swarm simulation** and **single robot testing**.

### Production Swarm Simulation (Recommended)

For full swarm behavior with 5 robots, execute these launch files in **separate terminals** in the following order:

#### Step 1: Launch Gazebo World
```bash
# Terminal 1: Start Gazebo Harmonic with flat ocean environment
ros2 launch swarm_bringup gazebo_world.launch.py
```
This launches:
- Gazebo Harmonic simulator.
- Flat ocean world environment.
- Clock synchronization bridge.

#### Step 2: Spawn Robot Swarm
```bash
# Terminal 2: Deploy 5 robots in formation (wait for Gazebo to fully load)
ros2 launch swarm_bringup spawn_robots.launch.py
```
This spawns:
- 5 aquatic surface robots (swarm_bot_1 to swarm_bot_5).
- Robot state publishers for each robot.
- Communication bridges between ROS2 and Gazebo.

#### Step 3: Activate Swarm Behavior
```bash
# Terminal 3: Start flocking controllers
ros2 launch swarm_bringup swarm_behavior.launch.py
```
This activates:
- Flocking behavior controllers for all robots.
- Inter-robot communication for swarm coordination.
- Autonomous navigation with collision avoidance.

#### Step 4: Data Collection (Optional)
```bash
# Terminal 4: Log swarm data for analysis
ros2 launch swarm_bringup data_logger.launch.py
```
This enables:
- Real-time data logging to CSV files.
- Velocity and position tracking.
- Automatic data summary generation.

### Single Robot Testing Mode
For development and testing purposes, use the simplified single robot launcher:
```bash
# Single terminal: Complete single robot test
ros2 launch swarm_bringup bringup_gz.launch.py
```
This test mode:
- Launches Gazebo with one robot.
- Applies smooth aquatic movement controller.
- Useful for debugging robot behavior.

### Controller Types
The project includes two main swarm controllers:
#### 1. Standard Flocking Controller (swarm_flocking)
**File**: swarm_control/swarm_flocking.py

Features:
- Complete flocking behavior (separation, alignment, cohesion).
- Laser-based obstacle avoidance.
- Inter-robot communication.
- Dynamic formation control.

**Use case**: Production swarm simulation with obstacle avoidance
```bash
# Launched automatically with swarm_behavior.launch.py
ros2 run swarm_control swarm_flocking --ros-args -p robot_id:=0
```

#### 2. Collision-Free Controller (swarm_flocking_without_collision)
**File**: swarm_control/swarm_flocking_without_collision.py
Features:
- Pure flocking behavior without collision detection.
- Lighter computational load.
- Smoother movement patterns.
- Simplified inter-robot dynamics.

**Use case**: Open environment simulation without obstacles
```bash
# Alternative controller for obstacle-free environments
ros2 run swarm_control swarm_flocking_without_collision --ros-args -p robot_id:=0
```

**Note** *: You must set the amount of robots you want to simulate in both controllers, also in the launch files (`~/swarm_bringup/spawn_robots.launch.py/`, `~/swarm_bringup/swarm_behavior.launch.py/`, `~/swarm_bringup/data_logger.launch.py/`). You even have to change them in the corresponding node in the folder `~/swarm_control/`*

### Launch File Parameters
#### Robot Naming Convention
- Robots are named: `swarm_bot_1`, `swarm_bot_2`, `...`, `swarm_bot_N`
- Namespaces: `/swarm/swarm_bot_X/`
- Topics follow pattern: `/swarm/swarm_bot_X/cmd_vel`, `/swarm/swarm_bot_X/odom`

### Monitoring Swarm Behavior
#### Real-time Monitoring
```bash
# View robot positions
ros2 topic echo /swarm/swarm_bot_1/odom

# Monitor velocities
ros2 topic list | grep cmd_vel

# Check swarm coordination
ros2 node list | grep swarm
```

#### Data Analysis
After running the simulation with data logger:
```bash
# Data saved to: ~/swarm_project/data/
ls ~/swarm_project/data/

# Files generated:
# - swarm_data_TIMESTAMP.csv (raw data)
# - swarm_data_TIMESTAMP_summary.txt (statistics)
```

### Visualization
#### Robot Model Visualization
```bash
# View robot URDF in RViz with joint controls
ros2 launch swarm_robot_description display.launch.py
```

#### Swarm Trajectory Visualization
```bash
# Launch RViz to visualize robot paths (while simulation runs)
rviz2
# Add topics: /swarm/swarm_bot_*/odom -> Path
```

### Stopping the Simulation
To properly stop the simulation:

1. Ctrl+C in Terminal 4 (data logger) - Generates final data summary.
2. Ctrl+C in Terminal 3 (swarm behavior).
3. Ctrl+C in Terminal 2 (spawn robots).
4. Ctrl+C in Terminal 1 (gazebo world) - Closes Gazebo.


### Expected Behavior
When running correctly, you should observe:

- Separation: Robots maintain minimum distance from neighbors.
- Alignment: Robots align their velocities with nearby robots.
- Cohesion: Robots are attracted to the center of local neighbors.
- Formation: Emergent flocking patterns and group coordination.
- Smooth Navigation: Aquatic-optimized movement dynamics.

### Performance Notes
- Recommended: 5-7 robots for optimal performance.
- CPU Usage: Monitor system resources during simulation.
- Gazebo Warnings: NodeShared::RecvSrvRequest() error warnings are normal and non-critical.
- Data Rate: ~50 data points per second per robot.

## Data Collection

The simulation automatically generates comprehensive datasets for swarm behavior analysis. All data is stored in `~/swarm_project/data/` with timestamped filenames.

### Data Files Generated

#### Raw Data (CSV Format)
**File**: `swarm_data_YYYYMMDD_HHMMSS.csv`

Contains real-time measurements with columns:
- `timestamp`: Simulation time (seconds)
- `robot_id`: Robot identifier (0-4 or more) you can set the amount of robots you want 
- `robot_name`: Robot namespace (`swarm_bot_1` to `swarm_bot_5`)
- `pos_x`, `pos_y`: Position coordinates (meters)
- `vel_x`, `vel_y`: Velocity components (m/s)
- `speed`: Total velocity magnitude (m/s)

#### Statistical Summary
**File**: `swarm_data_YYYYMMDD_HHMMSS_summary.txt`

Includes:
- Per-robot speed statistics (mean, max, min).
- Estimated total distance traveled.
- Position ranges and movement boundaries.
- Inter-robot distance distributions.
- Data collection metadata.

### Data Analysis Examples

#### Using Python (Basic)
```python
import csv
import matplotlib.pyplot as plt

# Load data
timestamps, speeds = [], []
with open('swarm_data_20250819_143022.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        if row['robot_id'] == '0':  # Analyze Robot 1
            timestamps.append(float(row['timestamp']))
            speeds.append(float(row['speed']))

# Plot speed over time
plt.plot(timestamps, speeds)
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.title('Robot 1 Speed Profile')
plt.show()
```
### Using Excel/LibreOffice Calc
1. Open the CSV file.
2. Create pivot tables grouping by robot_id.
3. Generate charts for velocity trends.
4. Analyze swarm cohesion patterns.

#### Typical Data Metrics
For a 5-robot and 300-second simulation:
- Data Points: ~15,000 total (3,000 per robot).
- File Size: ~1.2 MB CSV.
- Sampling Rate: ~10 Hz per robot.
- Velocity Range: 0.0 - 0.5 m/s (typical aquatic speeds).
- Position Range: -6 to +6 meters (bounded by world).

## Troubleshooting
Common Issues
### ROS2 Jazzy Not Found
```bash
# Verify ROS2 installation
ros2 --version
# Expected: ros2 doctor version 0.10.5

# If not found, reinstall ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop-full
```

### Gazebo Harmonic Version Conflicts
```bash
# Check Gazebo version
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# Remove conflicting versions
sudo apt remove gz-garden gz-fortress
sudo apt install gz-harmonic
```

### Gazebo Launch Failures
**Problem**: gz sim command not found
```bash
# Solution: Add Gazebo to PATH
echo "export PATH=/opt/ros/jazzy/opt/gz_sim_vendor/bin:$PATH" >> ~/.bashrc
source ~/.bashrc
```

**Problem**: World file not loading
```bash
# Check world file exists
ls ~/swarm_project/install/swarm_worlds/share/swarm_worlds/worlds/
# Should show: flat_ocean.sdf

# Verify package installation
colcon build --packages-select swarm_worlds
```

### Robot Spawning Issues
**Problem**: Robots not appearing in Gazebo
```bash
# Check if Gazebo service is running
gz service --list | grep create
# Expected: /world/flat_ocean/create

# Verify robot description
ros2 param get /swarm_bot_1/robot_state_publisher robot_description
```

**Problem**: "Host unreachable" errors
- Cause: Normal Gazebo service saturation with multiple robots.
- Impact: Non-critical warnings, simulation continues normally.
- Solution: Reduce update frequency or ignore warnings.

### Flocking Behavior Issues
**Problem**: Robots not flocking, moving individually
```bash
# Check inter-robot communication
ros2 topic list | grep odom
# Should show: /swarm/swarm_bot_X/odom for each robot

# Verify controllers are running
ros2 node list | grep flocking
# Expected: swarm_bot_X_flocking nodes
```

### Data Collection Issues
**Problem**: No data files generated
```bash
# Check data directory exists
ls ~/swarm_project/data/
# Create if missing:
mkdir -p ~/swarm_project/data/

# Verify data logger is running
ros2 node list | grep data_logger
```

**Note** *: We had a lot of issues with sensor plugins, mesh models (.STL) and other ROS2 and Gazebo Harmonic packages and resources.
We changed to a simplified URDF model and use a laser scan sensor plugin that doesn't work pretty well.
If you know how to fix it, please contact with us.*


### Getting help
If problems persist:

1. Check logs: ~/.ros/log/ for detailed error messages.
2. Verify versions: Ensure ROS2 Jazzy + Gazebo Harmonic compatibility.
3. Clean rebuild: Remove build/ and install/ directories, rebuild.
4. System resources: Ensure sufficient RAM (8GB+) and CPU capacity.

For additional support, include in your issue report:

- Operating system version (lsb_release -a)
- ROS2 version (ros2 --version)
- Gazebo version (gz sim --version)
- Error logs from ~/.ros/log/

### Mobile and Serial Robots - I PAO 2025
***Thank you and enjoy your simulation from our dedicated team!***
