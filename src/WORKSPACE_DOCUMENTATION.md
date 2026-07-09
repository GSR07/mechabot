# Mechabot ROS 2 Workspace Documentation

This workspace is a ROS 2 mobile robot project for a differential-drive robot named `mechabot`. It includes robot description, Gazebo simulation, ros2_control controllers, SLAM mapping, AMCL localization, Nav2 navigation, a serial hardware interface for an ESP32/Arduino-style controller, and Python scripts for sensing, docking, patrolling, cleaning, and RL-based battery-aware missions.

Workspace path:

```bash
/home/gsr/mechabot_ws/src
```

Typical parent workspace path:

```bash
/home/gsr/mechabot_ws
```

## Package Overview

| Package | Build type | Purpose |
| --- | --- | --- |
| `mechabot_bringup` | `ament_cmake` | High-level launch files that combine simulation, controllers, joystick, localization, RViz, and navigation. |
| `mechabot_description` | `ament_cmake` | Robot URDF/Xacro, meshes, Gazebo worlds, RViz config, and Gazebo model assets. |
| `mechabot_controller` | `ament_cmake` | ros2_control controller configuration, diff-drive controller, joint state broadcaster, joystick teleop, and twist mux config. |
| `mechabot_mapping` | `ament_cmake` | SLAM Toolbox launch/configuration and saved maps. |
| `mechabot_localization` | `ament_cmake` | Nav2 map server plus AMCL localization configuration and launch. |
| `mechabot_navigation` | `ament_cmake` | Nav2 controller, planner, smoother, behavior server, BT navigator, waypoint follower config, and waypoint script. |
| `mechabot_firmware` | `ament_cmake` | Real hardware `ros2_control` system plugin using serial communication through `libserial`. |
| `mechabot_scripts` | `ament_python` | Python nodes for sensor viewing, marker detection, docking, patrol, battery-aware cleaning, and RL docking/cleaning behavior. |

## Repository Structure

```text
.
├── mechabot_bringup/
│   └── launch/
├── mechabot_controller/
│   ├── config/
│   └── launch/
├── mechabot_description/
│   ├── launch/
│   ├── meshes/
│   ├── models/
│   ├── photos/
│   ├── rviz/
│   ├── urdf/
│   └── worlds/
├── mechabot_firmware/
│   ├── firmware/
│   ├── include/mechabot_firmware/
│   ├── launch/
│   └── src/
├── mechabot_localization/
│   ├── config/
│   ├── launch/
│   └── rviz/
├── mechabot_mapping/
│   ├── config/
│   ├── launch/
│   ├── maps/
│   └── rviz/
├── mechabot_navigation/
│   ├── behaviour_tree/
│   ├── config/
│   ├── launch/
│   └── mechabot_navigation/
└── mechabot_scripts/
    ├── mechabot_scripts/
    ├── resource/
    └── test/
```

## Build And Source

Run commands from the parent workspace, not from `src`.

```bash
cd /home/gsr/mechabot_ws
colcon build
source install/setup.bash
```

Build only selected packages:

```bash
cd /home/gsr/mechabot_ws
colcon build --packages-select mechabot_description mechabot_controller
source install/setup.bash
```

After editing launch files, config files, Python entry points, URDF/Xacro files, or C++ plugin code, rebuild and source again.

## Main Launch Commands

### Full Small-House Simulation With Localization And Navigation

This is the main integrated simulation launch.

```bash
ros2 launch mechabot_bringup simulated_robot.launch.py
```

It starts:

- Gazebo simulation using `small_house.world`.
- Robot state publisher.
- Robot spawn in Gazebo.
- Gazebo to ROS bridges for `/clock`, `/scan`, `/imu/out`, and `/camera/camera_info`.
- Camera image bridge for `/camera/image_raw`.
- ros2_control controller spawners.
- Joystick teleop.
- AMCL localization with map server.
- RViz using `mechabot_localization/rviz/global_localization.rviz`.
- Nav2 navigation stack.

Important code from `mechabot_bringup/launch/simulated_robot.launch.py`:

```python
gazebo = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("mechabot_description"),
        "launch",
        "gazebo.launch.py"
    ),
    launch_arguments={
        "world_name": "small_house"
    }.items()
)

global_localization = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("mechabot_localization"),
        "launch",
        "global_localization.launch.py"
    ),
)

navigation = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("mechabot_navigation"),
        "launch",
        "navigation.launch.py"
    ),
)
```

### QR Maze Simulation

```bash
ros2 launch mechabot_bringup qr_maze.launch.py
```

This starts:

- Gazebo with `qr_maze.world`.
- ros2_control controller spawners.

It does not start localization, navigation, joystick, or RViz.

### Gazebo Only

```bash
ros2 launch mechabot_description gazebo.launch.py world_name:=small_house
```

Available world names currently include:

- `empty`
- `small_house`
- `qr_maze`

Example:

```bash
ros2 launch mechabot_description gazebo.launch.py world_name:=qr_maze
```

### Robot Visualization Only

```bash
ros2 launch mechabot_description display.launch.py
```

Use this when checking the URDF/Xacro model in RViz without Gazebo.

### Controllers Only

```bash
ros2 launch mechabot_controller controller.launch.py
```

This spawns:

- `joint_state_broadcaster`
- `wheel_controller`

### Joystick Teleoperation

```bash
ros2 launch mechabot_controller joystick.launch.py use_sim_time:=true
```

Joystick teleop publishes to `input_joy/cmd_vel`, which is intended to feed into the velocity command path.

### SLAM Mapping

```bash
ros2 launch mechabot_mapping slam.launch.py use_sim_time:=true
```

Save a generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

For the integrated bringup file, SLAM is currently commented out. To map instead of localize/navigate, enable the `slam` launch include and disable `global_localization` and `navigation` in `mechabot_bringup/launch/simulated_robot.launch.py`.

### Localization

```bash
ros2 launch mechabot_localization global_localization.launch.py map_name:=small_house use_sim_time:=true
```

Use the QR maze map:

```bash
ros2 launch mechabot_localization global_localization.launch.py map_name:=qr_maze use_sim_time:=true
```

### Navigation

```bash
ros2 launch mechabot_navigation navigation.launch.py use_sim_time:=true
```

Navigation expects localization, map, TF, odometry, and scan data to already be available.

### Real Hardware Interface

```bash
ros2 launch mechabot_firmware hardware_interface.launch.py serial_port:=/dev/ttyUSB1
```

This starts:

- `robot_state_publisher`
- `ros2_control_node`
- real hardware interface using `mechabot_firmware/MechabotInterface`

Then spawn the controllers:

```bash
ros2 launch mechabot_controller controller.launch.py
```

## Robot Description

Main Xacro entry file:

```text
mechabot_description/urdf/mechabot.urdf.xacro
```

It includes:

```xml
<xacro:include filename="$(find mechabot_description)/urdf/mechabot_base.xacro" />
<xacro:include filename="$(find mechabot_description)/urdf/mechabot_sensors.xacro"/>
<xacro:include filename="$(find mechabot_description)/urdf/mechabot_gazebo.xacro"/>
<xacro:include filename="$(find mechabot_description)/urdf/mechabot_ros2control.xacro"/>
```

The robot accepts this argument:

```xml
<xacro:arg name="is_sim" default="true"/>
```

When `is_sim:=true`, Gazebo/Ignition ros2_control is used. When `is_sim:=false`, the real serial hardware plugin is used.

### Main Links And Joints

Important links:

- `base_footprint`
- `base_link`
- `right_wheel_link`
- `left_wheel_link`
- `front_right_castor_link`
- `front_left_castor_link`
- `rear_right_castor_link`
- `rear_left_castor_link`
- `toplid_link`
- `lidar_link`
- `camera_link`
- `camera_link_optical`
- `imu_link`

Important joints:

- `base_joint`, fixed from `base_footprint` to `base_link`.
- `right_wheel_joint`, continuous.
- `left_wheel_joint`, continuous.
- caster joints, fixed.
- `lidar_joint`, fixed.
- `camera_joint`, fixed.
- `camera_optical_joint`, fixed optical transform.
- `imu_joint`, fixed.

Wheel joints:

```xml
<joint name="right_wheel_joint" type="continuous">
    <origin xyz="0.0 -0.0925 0.0215" rpy="0.0 0.0 0.0"/>
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <axis xyz="0.0 1.0 0.0"/>
</joint>

<joint name="left_wheel_joint" type="continuous">
    <origin xyz="0.0 0.0925 0.0215" rpy="0.0 0.0 0.0"/>
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <axis xyz="0.0 1.0 0.0"/>
</joint>
```

## Gazebo Simulation

Main launch:

```text
mechabot_description/launch/gazebo.launch.py
```

The launch file:

- Reads `mechabot.urdf.xacro`.
- Passes `is_sim:=True`.
- Sets `GZ_SIM_RESOURCE_PATH` so Gazebo can find worlds and models.
- Starts `robot_state_publisher`.
- Starts `ros_gz_sim`.
- Spawns the robot from `robot_description`.
- Bridges sensor topics.

Robot spawn pose:

```text
x: 1.5
y: 5.18
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 1.57
```

Gazebo bridge topics:

```python
arguments=[
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
],
remappings=[
    ('/imu', '/imu/out'),
]
```

Camera image bridge:

```python
ros_gz_image_bridge = Node(
    package="ros_gz_image",
    executable="image_bridge",
    arguments=["/camera/image_raw"]
)
```

### Gazebo Sensors

Configured in:

```text
mechabot_description/urdf/mechabot_gazebo.xacro
```

LiDAR:

- Sensor type: `gpu_lidar`
- Topic: `scan`
- Frame: `lidar_link`
- Update rate: `5 Hz`
- Samples: `360`
- Range: `0.15 m` to `12.0 m`
- Noise stddev: `0.01`

IMU:

- Topic: `imu`, remapped to ROS as `/imu/out`
- Frame: `imu_link`
- Update rate: `100 Hz`

Camera:

- Topic: `camera/image_raw`
- Camera info topic: `camera/camera_info`
- Frame: `camera_link_optical`
- Resolution: `640x480`
- Update rate: `10 Hz`
- Horizontal FOV: `1.3962634`

## ros2_control And Controllers

Controller config:

```text
mechabot_controller/config/mechabot_controllers.yaml
```

Controller manager:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    activation_timeout: 15.0

    wheel_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

Diff-drive controller:

```yaml
wheel_controller:
  ros__parameters:
    use_stamped_vel: false
    position_feedback: false
    left_wheel_names: ['left_wheel_joint']
    right_wheel_names: ['right_wheel_joint']
    publish_rate: 100.0
    wheel_separation: 0.185
    wheel_radius: 0.034
    cmd_vel_timeout: 0.5
    base_frame_id: base_footprint
    enable_odom_tf: true
```

Velocity limits:

```yaml
linear:
  x:
    max_velocity: 2.0
    min_velocity: -2.0
    max_acceleration: 1.0
    min_acceleration: -1.0

angular:
  z:
    max_velocity: 2.5
    min_velocity: -2.5
    max_acceleration: 2.5
    min_acceleration: -2.5
```

Important controller topics usually include:

- `/cmd_vel` or muxed velocity command input, depending on active launch/config.
- `/wheel_controller/odom`
- `/joint_states`
- `/tf`
- `/tf_static`

Check active controllers:

```bash
ros2 control list_controllers
```

Check hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

Send a simple velocity command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Stop the robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Joystick Teleop

Main config:

```text
mechabot_controller/config/joy_teleop.yaml
```

Current mapping:

```yaml
joy_teleop:
  ros__parameters:
    move:
      type: topic
      interface_type: geometry_msgs/msg/Twist
      topic_name: input_joy/cmd_vel
      deadman_buttons: [7]
      axis_mappings:
        linear-x:
          axis: 3
          scale: 1.0
          offset: 0.0
        angular-z:
          axis: 0
          scale: 1.0
          offset: 0.0
```

Deadman button:

```text
button 7
```

Axes:

- Axis `3` controls forward/backward motion.
- Axis `0` controls yaw rotation.

## Mapping

SLAM launch:

```text
mechabot_mapping/launch/slam.launch.py
```

SLAM config:

```text
mechabot_mapping/config/slam_toolbox.yaml
```

Important parameters:

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping
    resolution: 0.05
    max_laser_range: 12.0
    use_scan_matching: true
    do_loop_closing: true
```

Saved maps:

```text
mechabot_mapping/maps/small_house/map.yaml
mechabot_mapping/maps/small_house/map.pgm
mechabot_mapping/maps/qr_maze/map.yaml
mechabot_mapping/maps/qr_maze/map.pgm
```

Map metadata:

```yaml
# small_house
image: map.pgm
resolution: 0.050000
origin: [-12.500000, -12.500000, 0.000000]
occupied_thresh: 0.65
free_thresh: 0.196

# qr_maze
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-9.07, -7.73, 0]
occupied_thresh: 0.65
free_thresh: 0.25
```

## Localization

Localization launch:

```text
mechabot_localization/launch/global_localization.launch.py
```

It starts:

- `nav2_map_server`
- `nav2_amcl`
- `nav2_lifecycle_manager`

Lifecycle nodes:

```python
lifecycle_nodes = ["map_server", "amcl"]
```

Default map:

```text
small_house
```

AMCL config:

```text
mechabot_localization/config/amcl.yaml
```

Important AMCL parameters:

```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: scan
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    min_particles: 500
    max_particles: 2000
    laser_max_range: 12.0
    laser_model_type: "likelihood_field"
    tf_broadcast: true
    set_initial_pose: true
```

Initial pose inside dock:

```yaml
initial_pose:
  x: 1.5
  y: 5.18
  z: 0.0
  yaw: 1.57
```

Publish a manual initial pose:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 1.5, y: 5.18, z: 0.0}, orientation: {z: 0.707, w: 0.707}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

## Navigation

Navigation launch:

```text
mechabot_navigation/launch/navigation.launch.py
```

It starts:

- `controller_server`
- `planner_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `lifecycle_manager_navigation`

Lifecycle nodes:

```python
lifecycle_nodes = [
    "controller_server",
    "planner_server",
    "smoother_server",
    "bt_navigator",
    "behavior_server",
    "waypoint_follower"
]
```

### Controller Server

Config:

```text
mechabot_navigation/config/controller_server.yaml
```

Important parameters:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    odom_topic: /wheel_controller/odom
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
```

Controller plugin:

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.15
  lookahead_dist: 0.35
  min_approach_linear_velocity: 0.05
  use_collision_detection: true
  use_rotate_to_heading: true
  allow_reversing: true
```

Local costmap:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      footprint: "[[-0.135, -0.115], [-0.135,  0.115], [0.135,  0.115], [0.135, -0.115]]"
      plugins: ["obstacle_layer", "inflation_layer"]
```

### Planner Server

Config:

```text
mechabot_navigation/config/planner_server.yaml
```

Planner plugin:

```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlanner2D"
  tolerance: 0.125
  allow_unknown: true
  max_iterations: 1000000
  max_planning_time: 2.0
  cost_travel_multiplier: 2.0
```

Global costmap:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### Behavior Server

Config:

```text
mechabot_navigation/config/behavior_server.yaml
```

Enabled behaviors:

```yaml
behavior_plugins: ["spin", "backup", "wait"]
```

### Smoother Server

Config:

```text
mechabot_navigation/config/smoother_server.yaml
```

Plugin:

```yaml
simple_smoother:
  plugin: "nav2_smoother::SimpleSmoother"
  tolerance: 1.0e-10
  max_its: 1000
  do_refinement: true
```

### BT Navigator

Config:

```text
mechabot_navigation/config/bt_navigator.yaml
```

Behavior tree:

```text
mechabot_navigation/behaviour_tree/simple_navigation_w_replanning_and_recovery.xml
```

### Waypoint Follower

Config:

```text
mechabot_navigation/config/waypoint_follower.yaml
```

Important parameters:

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 2
```

Python waypoint script:

```text
mechabot_navigation/mechabot_navigation/waypoint_following.py
```

It uses `nav2_simple_commander.BasicNavigator` and creates `PoseStamped` goals in the `map` frame.

Example goal creation code:

```python
def create_pose_stamped(self, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
        0.0, 0.0, rotation_z
    )
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose
```

Configured example waypoints:

```python
goal_pose0 = self.create_pose_stamped(5.0, 1.0, 0.0)
goal_pose1 = self.create_pose_stamped(0.0, 3.0, 0.0)
goal_pose2 = self.create_pose_stamped(3.0, 3.0, -1.57)
```

## Firmware And Real Hardware Interface

Package:

```text
mechabot_firmware
```

Plugin XML:

```text
mechabot_firmware/mechabot_interface.xml
```

C++ header:

```text
mechabot_firmware/include/mechabot_firmware/mechabot_interface.hpp
```

C++ implementation:

```text
mechabot_firmware/src/mechabot_interface.cpp
```

The hardware plugin implements:

```cpp
class MechabotInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
};
```

The real hardware block in Xacro:

```xml
<xacro:unless value="$(arg is_sim)">
    <hardware>
        <plugin>mechabot_firmware/MechabotInterface</plugin>
        <param name="port">/dev/ttyUSB1</param>
    </hardware>
</xacro:unless>
```

Serial settings:

```cpp
esp_.Open(port_);
esp_.SetBaudRate(LibSerial::BaudRate::BAUD_500000);
```

### Serial Protocol

The hardware interface sends right and left wheel velocity commands in this string format:

```text
r[p/n]VV.VV,l[p/n]VV.VV,
```

Example:

```text
rp05.30,ln12.45,
```

Meaning:

- `r` = right wheel.
- `l` = left wheel.
- `p` = positive direction.
- `n` = negative direction.
- Number = absolute wheel velocity command with two decimal places.

Write code:

```cpp
message_stream << std::fixed << std::setprecision(2)
  << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0))
  << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1))
  << ",";
```

Read code expects comma-separated wheel velocity feedback:

```cpp
if(res.at(0) == 'r')
{
  velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
  position_states_.at(0) += velocity_states_.at(0) * dt;
}
else if(res.at(0) == 'l')
{
  velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
  position_states_.at(1) += velocity_states_.at(1) * dt;
}
```

Firmware sketches:

```text
mechabot_firmware/firmware/mechabot_control_firmware/mechabot_control_firmware.ino
mechabot_firmware/firmware/encoder_cpr_counter/encoder_cpr_counter.ino
mechabot_firmware/firmware/encoder_cpr_counter_with_serial_input/encoder_cpr_counter_with_serial_input.ino
```

## Python Scripts

Package:

```text
mechabot_scripts
```

Registered console commands in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'read_lidar = mechabot_scripts.read_lidar:main',
        'read_imu = mechabot_scripts.read_imu:main',
        'read_camera = mechabot_scripts.read_camera:main',
        'detect_marker = mechabot_scripts.detect_marker:main',
        'maze_solver = mechabot_scripts.maze_solver:main',
        'auto_dock_undock = mechabot_scripts.auto_dock_undock:main',
        'docling_with_patrolling = mechabot_scripts.docling_with_patrolling:main',
        'battery_auto_dock=  mechabot_scripts.auto_docking_with_battery:main',
        'cleaning = mechabot_scripts.Cleaning_with_battery:main',
        'rl_nav = mechabot_scripts.rl_docking:main',
    ],
}
```

Run examples:

```bash
ros2 run mechabot_scripts read_lidar
ros2 run mechabot_scripts read_imu
ros2 run mechabot_scripts read_camera
ros2 run mechabot_scripts detect_marker
ros2 run mechabot_scripts auto_dock_undock
ros2 run mechabot_scripts docling_with_patrolling
ros2 run mechabot_scripts battery_auto_dock
ros2 run mechabot_scripts cleaning
ros2 run mechabot_scripts rl_nav
```

### Sensor Scripts

`read_lidar.py`:

- Reads LiDAR data.
- Uses `sensor_msgs/msg/LaserScan`.
- Intended topic is `/scan` or `scan`.

`read_imu.py`:

- Reads IMU data.
- Uses `sensor_msgs/msg/Imu`.
- Intended topic is `/imu/out`.

`read_camera.py`:

- Reads camera images.
- Uses `sensor_msgs/msg/Image`.
- Uses OpenCV/CV Bridge.
- Topic: `camera/image_raw`.

`detect_marker.py`:

- Reads `/camera/image_raw`.
- Detects visual markers/QR-like targets from camera data.

### Docking And Patrolling Scripts

`auto_dock_undock.py`:

- Node class: `SimpleDockingNode`.
- Publishes `geometry_msgs/msg/Twist` to `/cmd_vel`.
- Subscribes to:
  - `/camera/image_raw`
  - `/imu/out`
  - `scan`
- Uses camera, IMU, and LiDAR for undocking/docking.
- Uses a docking threshold around `0.15 m`.

Important subscriptions:

```python
self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
self.cam_sub = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10)
self.imu_sub = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)
self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
```

`docling_with_patrolling.py`:

- Combines docking/undocking with Nav2 waypoint patrol.
- Uses `BasicNavigator`.
- Publishes `/cmd_vel`.
- Subscribes to camera, IMU, and LiDAR.
- Navigates to waypoint lists, returns to a pre-dock position, then docks.

### Battery-Aware Docking

`auto_docking_with_battery.py`:

- Uses Nav2 and direct velocity control.
- Subscribes to:
  - `/camera/image_raw`
  - `/imu/out`
  - `/scan`
  - odometry
  - `/battery_status`
- Tracks battery state.
- Stops or returns when battery is critically low.
- Uses QR detection and LiDAR as final docking safety checks.

Important values visible in the script:

```python
self.max_final_docking_distance = 0.65
self.final_docking_speed = 0.050
self.docking_threshold = 0.160
self.battery = 100
```

### Cleaning And RL Navigation

`Cleaning_with_battery.py`:

- Large battery-aware cleaning behavior.
- Uses persisted memory in:

```text
mechabot_scripts/mechabot_scripts/cleaning_rl_memory.json
```

`rl_docking.py`:

- Main class: `VacuumCleanerWithBatteryRL`.
- Performs cleaning waypoints, battery monitoring, return-to-dock, charging, QR/lidar final docking, and RL action updates.
- Publishes battery values.
- Saves final battery state to:

```text
mechabot_scripts/mechabot_scripts/final_battery_reading.json
```

Important RL behavior:

- Loads Q-table from JSON memory.
- Builds state strings from waypoint position, front obstacle condition, bad zone flag, and battery band.
- Chooses actions with RL values.
- Updates rewards after navigation results.
- Generates cleaning waypoints from map data.
- Returns to dock and simulates/monitors charging when battery is low.

## Important Topics

Common ROS topics in this workspace:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time from Gazebo. |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan for SLAM, AMCL, Nav2 costmaps, docking logic. |
| `/imu/out` | `sensor_msgs/msg/Imu` | IMU data bridged from Gazebo. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Camera image stream. |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration/info stream. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Main velocity command topic used by scripts and controller path. |
| `input_joy/cmd_vel` | `geometry_msgs/msg/Twist` | Joystick velocity topic. |
| `/wheel_controller/odom` | `nav_msgs/msg/Odometry` | Diff-drive odometry. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint states from broadcaster/controller. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Map from map server or SLAM. |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Localized robot pose. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial pose input for AMCL. |
| `/battery_status` | commonly `std_msgs/msg/Int32` | Battery state used by battery-aware scripts. |

Inspect topics:

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic echo /amcl_pose --once
ros2 topic hz /scan
```

Inspect TF:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Important Frames

Expected frame chain:

```text
map -> odom -> base_footprint -> base_link -> sensor/wheel links
```

Important frame names:

- `map`
- `odom`
- `base_footprint`
- `base_link`
- `lidar_link`
- `camera_link`
- `camera_link_optical`
- `imu_link`

Nav2, SLAM, and AMCL are configured around:

```yaml
map_frame: map
odom_frame: odom
base_frame: base_footprint
scan_topic: /scan
```

## Common Workflows

### Start Full Simulation And Navigate

Terminal 1:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 launch mechabot_bringup simulated_robot.launch.py
```

Terminal 2:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 topic echo /amcl_pose --once
ros2 control list_controllers
```

Then set a Nav2 goal in RViz using `2D Goal Pose`.

### Run Mapping Instead Of Localization

Terminal 1:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 launch mechabot_description gazebo.launch.py world_name:=small_house
```

Terminal 2:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 launch mechabot_controller controller.launch.py
```

Terminal 3:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 launch mechabot_mapping slam.launch.py use_sim_time:=true
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/gsr/mechabot_ws/src/mechabot_mapping/maps/new_map/map
```

### Run A Docking Script

Start simulation/localization/navigation first:

```bash
ros2 launch mechabot_bringup simulated_robot.launch.py
```

Then run:

```bash
ros2 run mechabot_scripts auto_dock_undock
```

or:

```bash
ros2 run mechabot_scripts battery_auto_dock
```

or:

```bash
ros2 run mechabot_scripts rl_nav
```

### Run Real Hardware

Terminal 1:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 launch mechabot_firmware hardware_interface.launch.py serial_port:=/dev/ttyUSB1
```

Terminal 2:

```bash
cd /home/gsr/mechabot_ws
source install/setup.bash
ros2 launch mechabot_controller controller.launch.py
```

Terminal 3:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

## Checks And Debugging

### Build Check

```bash
cd /home/gsr/mechabot_ws
colcon build
source install/setup.bash
```

### Launch File Availability

```bash
ros2 launch mechabot_description gazebo.launch.py --show-args
ros2 launch mechabot_localization global_localization.launch.py --show-args
ros2 launch mechabot_navigation navigation.launch.py --show-args
```

### Controller Debugging

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic echo /wheel_controller/odom --once
```

If controllers are inactive:

```bash
ros2 control switch_controllers --activate wheel_controller joint_state_broadcaster
```

### Sensor Debugging

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic echo /imu/out --once
ros2 topic hz /camera/image_raw
```

### Nav2 Lifecycle Debugging

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

### Localization Debugging

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

If AMCL does not converge:

- Confirm `/scan` is publishing.
- Confirm `/map` is publishing.
- Confirm the initial pose matches the robot spawn pose.
- Confirm the map matches the Gazebo world.
- Use RViz `2D Pose Estimate`.

### Gazebo Resource Debugging

`gazebo.launch.py` sets `GZ_SIM_RESOURCE_PATH` to include the package parent and `mechabot_description/models`. If models or meshes are missing, check:

```bash
echo $GZ_SIM_RESOURCE_PATH
```

and confirm package installation after build:

```bash
ros2 pkg prefix mechabot_description
```

## Key Design Notes

- The workspace is simulation-first but includes a real hardware path through `mechabot_firmware`.
- The robot is differential-drive with two actuated wheel joints and passive caster geometry.
- Gazebo simulation uses `ign_ros2_control/IgnitionSystem`.
- Real hardware uses `mechabot_firmware/MechabotInterface`.
- Navigation uses Nav2 with Regulated Pure Pursuit and Smac 2D planner.
- Localization uses AMCL against maps stored in `mechabot_mapping/maps`.
- SLAM uses `sync_slam_toolbox_node`.
- The default integrated launch is configured for localization/navigation, not active SLAM.
- The default simulated world and default localization map are both `small_house`.
- The robot spawn pose and AMCL initial pose match: `x=1.5`, `y=5.18`, `yaw=1.57`.
- Docking scripts combine direct `/cmd_vel` control with sensor feedback, separate from pure Nav2 goals.
- Battery-aware scripts rely on battery topics/data and JSON memory files.

## Files Most Likely To Edit

| Goal | File |
| --- | --- |
| Change robot geometry | `mechabot_description/urdf/mechabot_base.xacro` |
| Change sensors | `mechabot_description/urdf/mechabot_sensors.xacro` and `mechabot_description/urdf/mechabot_gazebo.xacro` |
| Change simulation world/spawn | `mechabot_description/launch/gazebo.launch.py` |
| Change wheel controller tuning | `mechabot_controller/config/mechabot_controllers.yaml` |
| Change joystick controls | `mechabot_controller/config/joy_teleop.yaml` |
| Change SLAM behavior | `mechabot_mapping/config/slam_toolbox.yaml` |
| Change default map/localization | `mechabot_localization/launch/global_localization.launch.py` and `mechabot_localization/config/amcl.yaml` |
| Change Nav2 local control | `mechabot_navigation/config/controller_server.yaml` |
| Change Nav2 global planning | `mechabot_navigation/config/planner_server.yaml` |
| Change waypoint behavior | `mechabot_navigation/config/waypoint_follower.yaml` or `mechabot_navigation/mechabot_navigation/waypoint_following.py` |
| Change hardware serial port/protocol | `mechabot_firmware/launch/hardware_interface.launch.py`, `mechabot_description/urdf/mechabot_ros2control.xacro`, and `mechabot_firmware/src/mechabot_interface.cpp` |
| Change docking logic | `mechabot_scripts/mechabot_scripts/auto_dock_undock.py` or `mechabot_scripts/mechabot_scripts/auto_docking_with_battery.py` |
| Change RL cleaning/docking | `mechabot_scripts/mechabot_scripts/rl_docking.py` |

## Known Workspace Notes

- Many package manifests still contain placeholder descriptions and license fields:

```xml
<description>TODO: Package description</description>
<license>TODO: License declaration</license>
```

- Python cache files exist under `mechabot_scripts/mechabot_scripts/__pycache__/`; these are generated files and usually should not be committed.
- The `mechabot_description/models` directory is large and contains many Gazebo model assets.
- The navigation package directory is named `behaviour_tree` using British spelling.
- `battery_auto_dock` in `setup.py` has extra spaces around the module assignment, but it is still intended to point to `mechabot_scripts.auto_docking_with_battery:main`.

## Minimal Startup Cheat Sheet

```bash
# Build
cd /home/gsr/mechabot_ws
colcon build
source install/setup.bash

# Full sim with localization and navigation
ros2 launch mechabot_bringup simulated_robot.launch.py

# Check sensors
ros2 topic hz /scan
ros2 topic echo /imu/out --once
ros2 topic hz /camera/image_raw

# Check controllers
ros2 control list_controllers

# Check localization
ros2 topic echo /amcl_pose --once

# Run a script
ros2 run mechabot_scripts auto_dock_undock
```
