# Mobile Differential Drive Robot - SLAM Project

A ROS 2 workspace for 3D SLAM using RTABMap with a 6-wheel differential drive robot equipped with Intel RealSense D455 camera.

---

## Overview

This project implements **RGB-D SLAM** (Simultaneous Localization and Mapping) using RTABMap on a custom differential drive robot. The robot can build 3D maps of indoor environments and localize itself within those maps.

---

## Robot Specifications

### Hardware
- **Type**: 6-wheel differential drive robot
- **Camera**: Intel RealSense D455 (RGBD)
- **Sensors**: Wheel odometry, depth camera

### Software Stack
- **ROS 2**: Humble Hawksbill
- **SLAM**: RTABMap 0.22.1
- **Simulation**: Gazebo 11
- **Visualization**: RViz2, RTABMap Viz

---

## Quick Start

### Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# RTABMap
sudo apt install ros-humble-rtabmap-ros

# Gazebo
sudo apt install gazebo libgazebo-dev

# TurtleBot3 Gazebo (for worlds if required)
sudo apt install ros-humble-turtlebot3-gazebo

# Additional dependencies
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-xacro
```

### Installation

1. **Clone the repository**
```bash
git clone <repo-url>
cd ws_ddmobile
```

2. **Build the workspace**
```bash
colcon build --symlink-install
```

3. **Source the workspace**
```bash
source install/setup.bash
```

---

## Usage

### SLAM Mode (Mapping)

Launch SLAM mode to build a new map:

```bash
# Launch Gazebo, RTABMap SLAM, and RTABMap Viz
ros2 launch mobile_dd_robot rtabmap_slam_robust.launch.py

# Launch RViz2 separately
cd ws_ddmobile
source install/setup.bash
rviz2 -d src/mobile_dd_robot/config/rtabmap.rviz
```

### Localization Mode (Navigation)

Launch localization mode to use a pre-built map:

```bash
# Launch Gazebo, RTABMap Localization, and RTABMap Viz
ros2 launch mobile_dd_robot rtabmap_localization.launch.py

# Launch RViz2
cd ws_ddmobile
source install/setup.bash
rviz2 -d src/mobile_dd_robot/config/rtabmap.rviz
```

### Driving the Robot

Use keyboard teleop to drive the robot:

```bash
# Terminal 3: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

##  Package Structure

```
ws_ddmobile/
├── src/
│   └── mobile_dd_robot/
│       ├── launch/
│       │   ├── rtabmap_slam_robust.launch.py      # SLAM mode
│       │   └── rtabmap_localization.launch.py     # Localization mode
│       ├── model/
│       │   ├── robot_with_camera.xacro            # Robot URDF
│       │   ├── robot.gazebo                       # Gazebo plugins
│       │   └── camera.xacro                       # Camera model
│       ├── config/
│       │   └── rtabmap.rviz                       # RViz configuration
│       ├── CMakeLists.txt
│       └── package.xml
├── build/              (gitignored - build artifacts)
├── install/            (gitignored - install files)
├── log/                (gitignored - build logs)
└── README.md
```

---

## Configuration

### RTABMap Parameters

The launch files include optimized RTABMap parameters for robust SLAM:

#### Key Parameters
- **Grid Resolution**: 5cm cells for occupancy grid
- **Feature Detection**: GFTT (Good Features To Track) with 1200 features
- **Loop Closure**: Threshold 0.10 for moderate-aggressive detection
- **Ground Filtering**: Points below -10cm filtered out
- **Optimization**: 30 iterations with g2o optimizer

#### Parameter Highlights
```python
'Grid/CellSize': '0.05'                    # 5cm resolution
'Grid/MinGroundHeight': '-0.1'             # Filter underground artifacts
'Grid/MaxGroundHeight': '0.05'             # Ground detection
'Vis/MaxFeatures': '1200'                  # Visual features
'Rtabmap/LoopThr': '0.10'                  # Loop closure threshold
'Optimizer/Iterations': '30'               # Graph optimization
'RGBD/OptimizeMaxError': '3.0'             # Max optimization error
```
---

## Testing

### Verify Installation
```bash
# Check if RTABMap is installed
ros2 pkg list | grep rtabmap
```

### Run System Tests
1. Launch SLAM mode
2. Drive robot for 2-3 minutes
3. Check map is being built in RTABMap Viz
4. Verify occupancy grid in RViz2
5. Look for loop closures in console

---

### Building
```bash
cd ws_ddmobile
colcon build --symlink-install
```

### Testing Changes
```bash
# Rebuild single package
colcon build --symlink-install --packages-select mobile_dd_robot

# Source and test
source install/setup.bash
ros2 launch mobile_dd_robot rtabmap_slam_robust.launch.py
```

### Adding New Features
1. Modify launch files in `src/mobile_dd_robot/launch/`
2. Update robot model in `src/mobile_dd_robot/model/`
3. Rebuild: `colcon build --symlink-install`
4. Test in simulation

---

## ROS 2 Topics

### Published Topics
- `/odom` - Wheel odometry
- `/camera/d455/image_raw` - RGB image
- `/camera/d455/depth/image_raw` - Depth image
- `/camera/d455/camera_info` - Camera calibration
- `/camera/d455/points` - Point cloud
- `/map` - Occupancy grid map
- `/rtabmap/grid_prob_map` - Probability map

### Subscribed Topics
- `/cmd_vel` - Velocity commands for robot control
