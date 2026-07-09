# MechaBot - Autonomous Mobile Robot Navigation Platform

A comprehensive ROS 2-based autonomous mobile robot platform with advanced navigation, mapping, localization, and control capabilities. MechaBot is designed for research and development of autonomous navigation systems.

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Project Structure](#project-structure)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Package Documentation](#package-documentation)
- [Configuration](#configuration)
- [Usage Examples](#usage-examples)
- [Contributing](#contributing)
- [License](#license)

## 🤖 Overview

MechaBot is a complete ROS 2 ecosystem for autonomous mobile robot navigation and control. It includes:

- **URDF Description**: Complete robot model with sensors and actuators
- **Navigation Stack**: Path planning and autonomous navigation
- **Mapping & Localization**: SLAM-based mapping and AMCL localization
- **Motion Control**: Joint and velocity control for differential drive
- **Sensor Integration**: Camera, LiDAR, and IMU support
- **Gazebo Simulation**: Full simulation environment for testing
- **Autonomous Scripts**: Waypoint following, docking, and maze solving capabilities

## ✨ Key Features

- **ROS 2 Native**: Built for ROS 2 (Humble/Iron compatible)
- **Differential Drive Robot**: Two-wheel + four-castor configuration
- **Advanced Navigation**: BT-based navigation with recovery behaviors
- **SLAM Integration**: SLAM Toolbox for real-time mapping
- **Multi-Map Support**: Pre-configured maps for different environments
- **Joystick Control**: Teleoperation support with joy_teleop
- **Flexible Architecture**: Modular design for easy customization
- **Simulation Ready**: Complete Gazebo integration for development
- **Behavior Trees**: XML-based navigation behavior trees
- **Auto-Docking**: Automated docking and charging capabilities

## 📁 Project Structure

```
mechabot_ws/
├── src/
│   ├── mechabot_bringup/          # Launch configurations and startup scripts
│   ├── mechabot_controller/       # Motion and joystick control
│   ├── mechabot_description/      # URDF and robot model description
│   ├── mechabot_localization/     # AMCL localization setup
│   ├── mechabot_mapping/          # SLAM Toolbox configuration
│   ├── mechabot_navigation/       # Nav2 navigation stack
│   └── mechabot_scripts/          # Autonomous behavior scripts
├── build/                          # Build artifacts (generated)
├── install/                        # Installation artifacts (generated)
└── README.md                       # This file
```

## 🔧 System Requirements

### Hardware
- **Processor**: Intel i5/i7 or equivalent (or ARM for embedded)
- **RAM**: Minimum 4GB (8GB+ recommended)
- **Storage**: 20GB for full development environment
- **Sensors**: LiDAR, Camera, IMU (optional but recommended)

### Software
- **OS**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **ROS 2**: Humble or Iron distribution
- **Python**: 3.10+
- **Build Tools**: colcon, cmake, g++
- **Simulation**: Gazebo 11.10+ (for simulation only)

### Dependencies
```bash
# Core ROS 2 packages
sudo apt install ros-${ROS_DISTRO}-ros2-control
sudo apt install ros-${ROS_DISTRO}-ros2-controllers
sudo apt install ros-${ROS_DISTRO}-nav2-*
sudo apt install ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-gazebo-ros

# Additional tools
sudo apt install python3-colcon-common-extensions
```

## 📦 Installation

### 1. Setup Workspace

```bash
# Create workspace
mkdir -p ~/mechabot_ws/src
cd ~/mechabot_ws

# Clone repository (if not already cloned)
cd src
git clone https://github.com/GSR07/mechabot.git .
cd ..
```

### 2. Install Dependencies

```bash
# Using rosdep
sudo rosdep init  # (Skip if already initialized)
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# For specific ROS distribution (replace humble with your distro)
source /opt/ros/humble/setup.bash
```

### 3. Build the Workspace

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select mechabot_description

# Build with verbose output (for debugging)
colcon build --event-handlers console_direct+
```

### 4. Source the Setup Files

```bash
# Source the setup.bash
source install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/mechabot_ws/install/setup.bash" >> ~/.bashrc
```

## 🚀 Quick Start

### Launch the Robot in Simulation

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch mechabot_bringup simulated_robot.launch.py

# Terminal 2: Launch navigation
ros2 launch mechabot_navigation navigation.launch.py

# Terminal 3: Launch RViz with pre-configured layout
rviz2 -d install/mechabot_description/share/mechabot_description/rviz/display.rviz
```

### Teleoperation with Joystick

```bash
# Launch joystick controller
ros2 launch mechabot_controller joystick.launch.py
```

### Autonomous Navigation

```bash
# Navigate to a goal using Nav2
# Use RViz to set 2D Nav Goal
# Or use command line:
ros2 action send_goal nav2_msgs/action/NavigateToPose navigate_to_pose "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 📚 Package Documentation

### mechabot_bringup
Startup and configuration package for launching robot systems.

**Launchfiles:**
- `simulated_robot.launch.py` - Launch simulated robot with Gazebo
- `qr_maze.launch.py` - Launch with QR maze environment

**Configuration:**
- Robot parameters and tf settings
- Sensor configurations

### mechabot_controller
Motion control and teleoperation package.

**Nodes:**
- Joy controller for teleoperation
- Twist multiplexer for command arbitration

**Configuration Files:**
- `mechabot_controllers.yaml` - Controller configurations
- `joy_teleop.yaml` - Joystick button mappings
- `twist_mux_*.yaml` - Command priority settings

### mechabot_description
Robot model and physical description package.

**Contents:**
- **URDF/Xacro Files**: Complete robot model definition
  - `mechabot.urdf.xacro` - Main robot model
  - `mechabot_base.xacro` - Base platform definition
  - `mechabot_sensors.xacro` - Sensor configurations
  - `mechabot_ros2control.xacro` - Hardware interface
  - `mechabot_gazebo.xacro` - Gazebo plugins and physics

- **Meshes**: STL geometry for visualization and collision
- **RViz**: Pre-configured visualization layouts
- **Examples**: Template URDF for reference

### mechabot_localization
Localization package using AMCL algorithm.

**Configuration:**
- `amcl.yaml` - AMCL particle filter parameters

**Launchfiles:**
- `global_localization.launch.py` - Global localization setup

**Usage:**
```bash
ros2 launch mechabot_localization global_localization.launch.py
```

### mechabot_mapping
SLAM-based mapping package using SLAM Toolbox.

**Configuration:**
- `slam_toolbox.yaml` - SLAM parameters and settings

**Launchfiles:**
- `slam.launch.py` - Launch SLAM mapping node

**Maps:**
Pre-configured maps for different environments:
- `maps/qr_maze/` - QR code based maze environment
- `maps/small_house/` - Small house environment

```bash
ros2 launch mechabot_mapping slam.launch.py
```

### mechabot_navigation
Nav2-based navigation stack with behavior trees.

**Configuration:**
- `bt_navigator.yaml` - Behavior tree configuration
- `controller_server.yaml` - Path tracking controller
- `planner_server.yaml` - Path planner (DWB)
- `smoother_server.yaml` - Path smoothing
- `behavior_server.yaml` - Behavior server settings
- `waypoint_follower.yaml` - Waypoint following configuration

**Behavior Trees:**
- `simple_navigation_w_replanning_and_recovery.xml` - Main behavior tree

**Launchfiles:**
- `navigation.launch.py` - Launch complete navigation stack

```bash
ros2 launch mechabot_navigation navigation.launch.py
```

### mechabot_scripts
Autonomous behavior and utility scripts.

**Scripts:**
- `waypoint_following.py` - Follow predefined waypoints
- `auto_dock_undock.py` - Autonomous docking/undocking
- `auto_docking_with_battery.py` - Docking based on battery level
- `maze_solver.py` - Autonomous maze solving algorithm
- `detect_marker.py` - QR/ArUco marker detection
- `read_camera.py` - Camera feed processing
- `read_lidar.py` - LiDAR data processing
- `read_imu.py` - IMU sensor reading
- `rl_docking.py` - RL-based docking strategy

## ⚙️ Configuration

### Robot Parameters

Edit `mechabot_description/urdf/mechabot_base.xacro`:

```xml
<!-- Robot dimensions -->
<xacro:arg name="base_length" default="0.35" />
<xacro:arg name="base_width" default="0.25" />
<xacro:arg name="wheel_radius" default="0.05" />
<xacro:arg name="wheel_separation" default="0.20" />
```

### Navigation Parameters

Edit `mechabot_navigation/config/bt_navigator.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    bt_xml_filename: "simple_navigation_w_replanning_and_recovery.xml"
    default_server_timeout: 20
```

### Controller Parameters

Edit `mechabot_controller/config/mechabot_controllers.yaml`:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
```

## 💡 Usage Examples

### Example 1: Basic Navigation in Simulation

```bash
# Terminal 1: Start simulation
ros2 launch mechabot_bringup simulated_robot.launch.py

# Terminal 2: Start navigation
ros2 launch mechabot_navigation navigation.launch.py

# Terminal 3: Set a navigation goal
ros2 launch mechabot_scripts waypoint_following.py
```

### Example 2: Mapping a New Environment

```bash
# Terminal 1: Start Gazebo with robot
ros2 launch mechabot_bringup simulated_robot.launch.py

# Terminal 2: Start SLAM
ros2 launch mechabot_mapping slam.launch.py

# Terminal 3: Teleoperate robot to map area
ros2 launch mechabot_controller joystick.launch.py

# Terminal 4: Save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SerializePoseGraph "filename: maps/my_map"
```

### Example 3: Autonomous Docking

```bash
# Launch all necessary components
ros2 launch mechabot_bringup simulated_robot.launch.py
ros2 launch mechabot_navigation navigation.launch.py

# Run docking script
ros2 run mechabot_scripts auto_docking_with_battery
```

## 🔗 ROS 2 Ecosystem Integration

MechaBot integrates with key ROS 2 packages:

- **Nav2**: Navigation library for autonomous navigation
- **SLAM Toolbox**: Graph-based SLAM
- **ROS2 Control**: Hardware abstraction for motion control
- **ROS2 Controllers**: High-level motion control
- **Gazebo**: Physics-based simulation
- **RViz2**: 3D visualization and monitoring

## 📊 Topics and Services

### Published Topics
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry (nav_msgs/Odometry)
- `/tf` - Transform frames
- `/scan` - LiDAR scans (sensor_msgs/LaserScan)
- `/camera/image_raw` - Camera images
- `/imu/data` - IMU data

### Subscribed Topics
- `/cmd_vel_from_nav` - Navigation velocity commands
- `/cmd_vel_from_joy` - Joystick velocity commands

### Services
- `/slam_toolbox/save_map` - Save current SLAM map
- `/nav2_costmap/clear_around_robot` - Clear costmap
- `/nav2_motion_primitives` - Motion primitive library

## 🐛 Troubleshooting

### Build Issues
```bash
# Clean build
colcon clean cache
colcon build --force-cmake-configure

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Runtime Issues
```bash
# Check if all nodes are running
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /topic_name

# View transforms
ros2 run tf2_tools view_frames
```

### Simulation Issues
```bash
# Check Gazebo connection
gz topic -l

# Verify robot spawned
ros2 node list | grep gazebo
```

## 📖 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ROS 2 Control](https://control.ros.org/)

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see LICENSE file for details.

## ✉️ Contact & Support

For issues, questions, or suggestions:
- Open an issue on GitHub
- Check existing documentation
- Refer to ROS 2 community resources

## 🎯 Future Enhancements

- [ ] Multi-robot coordination
- [ ] Advanced manipulation capabilities
- [ ] Deep learning-based perception
- [ ] Real-time performance optimization
- [ ] Hardware abstraction layer improvements
- [ ] Additional simulation environments

---

**Last Updated**: June 25, 2026

**Maintained by**: GSR07

**Repository**: [GitHub - MechaBot](https://github.com/GSR07/mechabot)
