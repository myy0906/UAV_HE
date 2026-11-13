# UAV_HE — Development Environment and Dependencies

## 1. Project Overview

UAV_HE is a UAV Vision-Language Navigation (UAV-VLN)
simulation project.

The system integrates:

- Natural language task understanding
- Vision-based target detection
- RGB-D target localization
- A* path planning
- Safe waypoint generation
- ROS 2 task coordination
- PX4 Offboard flight control
- Gazebo software-in-the-loop simulation

The validated baseline was completed on
November 11, 2026.

---

## 2. Operating Systems

### 2.1 Windows Host

The Windows host is responsible for:

- Running LM Studio
- Loading the Qwen3-VL model
- Providing the model inference API
- Running VMware Workstation

Exact Windows, VMware and LM Studio versions
should be recorded separately on the host.

### 2.2 Ubuntu Virtual Machine

| Component | Version |
|---|---|
| Operating System | Ubuntu 24.04.5 LTS |
| ROS | ROS 2 Jazzy |
| Gazebo | Gazebo Harmonic |
| PX4 | PX4-Autopilot v1.17.0 |
| Python | See installed version below |

The project was developed and tested in
an Ubuntu virtual machine.

---

## 3. AI Model and Inference Environment

| Component | Configuration |
|---|---|
| Inference software | LM Studio |
| Vision-Language Model | Qwen3-VL 8B |
| Model identifier | qwen/qwen3-vl-8b |
| API type | OpenAI-compatible API |
| Default API port | 1234 |
| Deployment platform | Windows host |

The model is hosted on Windows.

The Ubuntu virtual machine communicates
with LM Studio through the virtual network.

The API host address depends on the
individual developer's network configuration.

Model weights are not included in this repository.

Important:

In the current implementation, Qwen3-VL is
primarily used for language task understanding
and structured mission generation.

Red target detection is implemented using
OpenCV HSV-based image processing.

Three-dimensional target localization uses
RGB-D sensor data.

---

## 4. ROS 2 Environment

ROS distribution:

    jazzy

Main workspace:

    ~/px4_ros2_ws

Project packages:

    uav_vln_interfaces
    uav_vln_px4
    uav_vln_perception
    uav_vln_mapping
    uav_vln_navigation
    uav_vln_llm
    uav_vln_mission
    uav_vln_evaluation
    uav_vln_bringup

Additional dependency:

    px4_msgs

Main ROS-related dependencies include:

- rclpy
- rclcpp
- rosidl
- sensor_msgs
- geometry_msgs
- nav_msgs
- cv_bridge
- ros_gz_bridge
- px4_msgs

Not every package listed above is a direct
dependency of every project module.

---

## 5. PX4 and Gazebo Environment

PX4 source directory:

    ~/PX4-Autopilot

PX4 baseline:

    v1.17.0

Simulation vehicle:

    X500 with depth camera / lidar model

PX4 SITL target:

    gz_x500_depth_lidar

The final UAV-VLN architecture uses
RGB-D perception and a known static map
for path planning.

It does not depend on online LiDAR-based
occupancy-grid construction.

The original simulation environment includes
custom vehicle configurations and a custom
airframe configuration.

These customizations must be restored when
recreating the complete simulation environment.

PX4 communicates with ROS 2 using
Micro XRCE-DDS Agent.

---

## 6. Main Python Dependencies

The project uses Python packages including:

- numpy
- opencv-python / system OpenCV bindings
- jsonschema
- matplotlib
- requests
- ROS 2 Python packages
- cv_bridge

The exact installed versions of major packages
are recorded in the version snapshot below.

Use the Ubuntu and ROS 2 package manager
where appropriate.

Avoid blindly upgrading system ROS packages
with pip.

---

## 7. Coordinate Systems

Upper-level navigation:

    ENU (East-North-Up)

PX4 native navigation:

    NED (North-East-Down)

Coordinate conversion is handled at the
PX4 execution interface.

Gazebo ground-truth information is used
for offline evaluation and validation.

It must not be substituted for RGB-D target
localization during autonomous navigation.

---

## 8. Main Configuration Files

Important project files:

    src/uav_vln_bringup/config/known_map.json

    src/uav_vln_bringup/config/gz_bridge.yaml

    src/uav_vln_bringup/config/mission.schema.json

    src/uav_vln_bringup/launch/full_vln.launch.py

    src/uav_vln_navigation/uav_vln_navigation/waypoint_compiler.py

    src/uav_vln_mission/uav_vln_mission/mission_manager.py

The known-map configuration defines:

- Map resolution
- Geofence boundaries
- Static obstacles
- Safety clearance
- Speed limits
- Waypoint acceptance threshold

---

## 9. Build Instructions

Source the ROS 2 environment:

```bash
source /opt/ros/jazzy/setup.bash
```

Enter the workspace:

```bash
cd ~/px4_ros2_ws
```

Install available ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the ROS 2 project:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/px4_ros2_ws/install/setup.bash
```

Note:

These commands assume that ROS 2, PX4,
Gazebo, the appropriate px4_msgs definitions
and custom PX4 configurations are already installed.

---

## 10. Runtime Startup Order

### Step 1: Windows

Start LM Studio.

Load:

    qwen/qwen3-vl-8b

Start the local inference server on port 1234.

### Step 2: Ubuntu — Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Step 3: Ubuntu — PX4 SITL

```bash
cd ~/PX4-Autopilot

PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
make px4_sitl gz_x500_depth_lidar
```

### Step 4: Ubuntu — ROS 2 System

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash

ros2 launch uav_vln_bringup full_vln.launch.py
```

### Step 5: Ubuntu — Mission Interface

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash

ros2 run uav_vln_mission cli_console
```

Use:

    /status

to check whether the mission action server
is available.

---

## 11. Simulation Results

Default results directory:

    ~/px4_ros2_ws/results/

Default logs directory:

    ~/px4_ros2_ws/logs/

The project records:

- Original natural-language instruction
- Model output
- Candidate semantic plan
- Validation report
- Safe waypoint plan
- RGB-D target localization
- Flight trajectory
- Task-state events
- Mission report
- Performance metrics

The repository may include representative
simulation results.

---

## 12. Known Limitations

The current validated implementation:

- Uses a known static simulation environment
- Uses predefined obstacle geometry
- Supports a constrained mission protocol
- Does not provide general unknown-scene SLAM
- Does not perform online dynamic-obstacle avoidance
- Has not been validated through real UAV flight

---

## 13. Environment Reproduction Notes

1. Match the Ubuntu and ROS 2 distributions.
2. Use a compatible PX4 and px4_msgs version.
3. Restore the custom PX4 model and airframe.
4. Configure Gazebo Harmonic and ROS bridges.
5. Install required Python and ROS packages.
6. Configure the local Qwen3-VL model service.
7. Check the model API address.
8. Build the ROS workspace.
9. Verify sensor, DDS and PX4 connectivity.
10. Run the complete simulation.

---

## 14. Installed Version Snapshot

The following section is generated from the
actual Ubuntu development environment.


### Ubuntu Operating System

```text
Distributor ID:	Ubuntu
Description:	Ubuntu 24.04.5 LTS
Release:	24.04
Codename:	noble
```

### Linux Kernel

```text
7.0.0-43-generic
```

### Python

```text
Python 3.12.3
```

### Git

```text
git version 2.43.0
```

### ROS Distribution

```text
jazzy
```

### PX4 Source Revision

```text
v1.17.0-dirty
d6f12ad1c4f70ad3230afd7d86e971421e02fef4
```

### Installed ROS Packages

```text
ros-jazzy-ros-gz-bridge: 1.0.24-1noble.20260920.234644
ros-jazzy-cv-bridge: 4.1.0-1noble.20260911.143610
ros-jazzy-rclpy: 7.1.12-1noble.20260912.162354
ros-jazzy-sensor-msgs: 5.3.8-1noble.20260911.140426
ros-jazzy-geometry-msgs: 5.3.8-1noble.20260911.102920
ros-jazzy-nav-msgs: 5.3.8-1noble.20260911.110342
```

### Python Package Versions

```text
numpy 1.26.4
matplotlib 3.6.3
opencv-python not found in Python package metadata
jsonschema 4.10.3
requests 2.31.0
scipy 1.11.4
```

### Gazebo CLI

```text
The 'gz' command provides a command line interface to the Gazebo Tools.

  gz <command> [options]

List of available commands:

  help:          Print this help text.
  fuel:          Manage simulation resources.
  gui:           Launch graphical interfaces.
  log:           Record or playback topics.
  model:         Print information about models.
  msg:           Print information about messages.
  param:         List, get or set parameters.
  plugin:        Print information about plugins.
  sdf:           Utilities for SDF files.
  service:       Print information about services.
  sim:           Run and manage the Gazebo Simulator.
  topic:         Print information about topics.

Options:

  --force-version <VERSION>  Use a specific library version.
  --versions                 Show the available versions.
  --commands                 Show the available commands.
Use 'gz help <command>' to print help for a command.
```

### Micro XRCE-DDS Agent

```text
/usr/local/bin/MicroXRCEAgent
```
