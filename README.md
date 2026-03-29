# Home-Tidying Robot Simulation

A complete ROS 2 + Gazebo simulation of a home-tidying robot. The robot navigates a two-room home, detects small objects, picks them up, and returns them to a collection box.

> **Demo Video**: _[Link will be added after recording]_

---

## Overview

| Component | Description |
|-----------|-------------|
| **Robot model** | 4-wheeled skid-steer base, torso, 3-DOF actuated arm + gripper, cosmetic second arm, LED face panel, forward camera, 360° LiDAR |
| **Home world** | Two-room home with doorway, furniture, 6 pickup objects, collection box |
| **Tidying task** | Autonomous waypoint navigation + LiDAR obstacle avoidance, full pick-and-place pipeline |
| **ROS 2** | Humble |
| **Simulator** | Gazebo (Ignition Fortress / `ros-humble-ros-gz`) |

---

## Required Environment

| Component | Requirement |
|-----------|-------------|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Simulator | Gazebo via `ros-humble-ros-gz` (Ignition Fortress) |
| Build | colcon |
| Python | 3.10+ |

---

## Installation

### 1. Install ROS 2 Humble

Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) if not already installed.

```bash
# Verify ROS 2 is installed
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. Install System Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep
```

### 3. Initialize rosdep (if not done)

```bash
sudo rosdep init   # skip if already done
rosdep update
```

### 4. Clone the Repository

```bash
mkdir -p ~/tidybot_ws/src
cd ~/tidybot_ws/src
git clone https://github.com/<your-username>/Home-Tidying-Robot.git .
```

### 5. Install Python Dependencies

```bash
pip3 install -r ~/tidybot_ws/src/requirements.txt
```

### 6. Install ROS Dependencies

```bash
cd ~/tidybot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 7. Build the Workspace

```bash
cd ~/tidybot_ws
colcon build --symlink-install
```

A successful build produces output like:
```
Summary: 1 package finished [Xs]
```

### 8. Source the Workspace

```bash
source ~/tidybot_ws/install/setup.bash
```

Add this to your `~/.bashrc` for convenience:
```bash
echo "source ~/tidybot_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the Simulation

### Single Launch Command

```bash
ros2 launch tidybot_sim tidy.launch.py
```

This single command:
1. Launches Gazebo with the two-room home world
2. Spawns the tidybot robot at position `(0.5, 0.0, 0.13)`
3. Starts `robot_state_publisher` and the ROS-Gazebo bridge
4. Launches the autonomous navigator node (after a 5-second startup delay)
5. Launches the arm controller node for pick-and-place
6. Starts the odometry logger — outputs CSV to `~/tidybot_log_<timestamp>.csv`

### Expected Output

In the terminal you should see:
```
[navigator]: Starting autonomous navigation — 11 waypoints loaded
[navigator]: Waypoint 1/11: target=(2.0, 0.0), distance=1.5m
[logger]:    Opened log file: ~/tidybot_log_20260329_120000.csv
...
[navigator]: Entered Room 2 (x > 5.5m)
...
[navigator]: Navigation complete. Distance traveled: 24.3m
[logger]:    Rooms visited: Room1, Room2 | Total distance: 24.3m
```

### Optional: Visualize in RViz

```bash
# In a separate terminal (with workspace sourced)
rviz2 -d ~/tidybot_ws/src/config/tidybot.rviz
```

### Optional: Monitor Sensor Topics

```bash
# Camera images
ros2 topic hz /camera/image_raw

# LiDAR scans
ros2 topic hz /scan

# Odometry
ros2 topic echo /odom

# Robot status
ros2 topic echo /tidybot/status
```

---

## Package Structure

```
src/tidybot_sim/
├── package.xml                     # ROS 2 package manifest
├── setup.py                        # Python package install config
├── setup.cfg
├── resource/
│   └── tidybot_sim                 # ament resource marker
├── description/
│   ├── tidybot.urdf.xacro          # Top-level robot entry point
│   ├── base.xacro                  # 4-wheel skid-steer base + DiffDrive plugin
│   ├── torso_and_sensors.xacro     # Torso, camera, LiDAR, face panel
│   ├── right_arm.xacro             # 3-DOF actuated arm + parallel jaw gripper
│   └── left_arm.xacro              # Cosmetic mirror arm (static joints)
├── worlds/
│   └── home.sdf                    # Two-room home world
├── config/
│   ├── bridge.yaml                 # ROS↔Gazebo topic bridge configuration
│   └── tidybot.rviz                # RViz configuration
├── launch/
│   └── tidy.launch.py              # Single launch entry point
├── tidybot_sim/
│   ├── __init__.py
│   ├── navigator.py                # Waypoint nav + LiDAR obstacle avoidance
│   ├── arm_controller.py           # Arm joint control + pick-and-place
│   └── logger.py                   # Odometry CSV logger
└── docs/
    └── APPROACH.md                 # Design decisions, tradeoffs, Drift comparison
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                     │
│  ┌──────────────┐  ┌──────────────────────────────────┐ │
│  │  home.sdf    │  │         tidybot robot            │ │
│  │  - Room 1    │  │  ┌──────────────────────────┐    │ │
│  │  - Room 2    │  │  │  DiffDrive plugin        │    │ │
│  │  - Furniture │  │  │  Sensors system          │    │ │
│  │  - 6 objects │  │  │  JointPositionController │    │ │
│  │  - Coll. box │  │  │  DetachableJoint (×6)    │    │ │
│  └──────────────┘  │  └──────────────────────────┘    │ │
└────────────────────┼─────────────────────────────────── ┘
                     │  ros_gz_bridge
           ┌─────────▼──────────────────────┐
           │         ROS 2 Topics           │
           │  /cmd_vel    /odom             │
           │  /scan       /camera/image_raw │
           │  /joint_states  /clock         │
           │  /arm/*_cmd  /gripper/attach_* │
           └─────┬──────────┬──────────────┘
                 │          │
    ┌────────────▼──┐  ┌────▼──────────────┐  ┌──────────┐
    │  navigator.py │  │ arm_controller.py │  │ logger.py│
    │               │  │                   │  │          │
    │  Waypoint nav │  │  Joint sequences  │  │  CSV log │
    │  LiDAR avoid  │  │  Attach/detach    │  │  Odom    │
    └───────────────┘  └───────────────────┘  └──────────┘
```

### Robot Design

```
        [LED Face Panel]
             |||
        ┌────────────┐
        │   Torso    │◄── Camera (forward-facing)
   Left │            │ Right  ◄── LiDAR (360°, top)
   Arm  │            │ Arm (3-DOF: shoulder/elbow/wrist)
(static)│            │ + Parallel Jaw Gripper
        └────────────┘
        ┌────────────┐
        │    Base    │   0.50m × 0.48m × 0.12m
FL ─────┤            ├───── FR    (skid-steer)
RL ─────┤            ├───── RR
        └────────────┘
```

### Navigation Strategy

The navigator uses a **waypoint + reactive obstacle avoidance** approach:

1. **Pre-defined waypoints** trace a path through Room 1 → doorway → Room 2 → return
2. **Heading control**: rotate in place until aligned with next waypoint (< 0.1 rad error)
3. **Drive to waypoint**: move forward until within 0.25m of target
4. **Obstacle avoidance**: if any LiDAR beam in the forward ±30° sector reads < 0.5m, rotate left at 0.4 rad/s for 1.5 seconds
5. **Pick-and-place trigger**: when within 0.5m of a known object position, pause navigation and signal arm controller

### Pick-and-Place Pipeline (Bonus)

1. Navigator signals arm controller with object index via `/arm/trigger_pickup`
2. Arm executes **PICKUP_SEQUENCE**: open gripper → reach down → close gripper → lift
3. **DetachableJoint** plugin attaches object to gripper link
4. Navigator resumes, returns to collection box position
5. Arm executes **PLACE_SEQUENCE**: reach toward box → open gripper → retract
6. Object detaches and falls into box

---

## Home World Layout

```
┌─────────────────────────┬──────────────────────────┐
│       ROOM 1            │        ROOM 2            │
│    (5.0m × 4.5m)        │     (4.5m × 4.5m)        │
│                         │                           │
│  [Collection Box]       │          [Shelf]          │
│  ★ Robot Start          │                           │
│                         │                           │
│     ● obj_red           │   ● obj_yellow            │
│     ● obj_green    ════ │   ● obj_purple            │
│     ● obj_blue   doorway│   ● obj_orange            │
│                         │                           │
│  [Table][Chairs]        │  [Table][Chairs]          │
│  [Couch]                │                           │
└─────────────────────────┴──────────────────────────┘
  ● = pickup object (dynamic, on floor)
  ★ = robot start position (0.5, 0.0)
```

---

## Measurable Output

The simulation produces three forms of measurable output:

1. **CSV Log** at `~/tidybot_log_<timestamp>.csv`
   - Columns: `sim_time, x, y, z, yaw_deg, linear_x, angular_z, min_lidar_range`
   - Written at 20 Hz from `/odom` messages

2. **Terminal summary** on shutdown:
   ```
   Total distance traveled: XX.X m
   Rooms visited: Room1, Room2
   Objects collected: N/6
   Simulation time: XX.X s
   ```

3. **Published sensor topics** (verifiable via `ros2 topic hz`):
   - `/camera/image_raw` at ~15 Hz
   - `/scan` at ~10 Hz
   - `/odom` at ~20 Hz

---

## Configuration

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `home.sdf` | Path to world SDF file |
| `robot_x` | `0.5` | Robot spawn X position |
| `robot_y` | `0.0` | Robot spawn Y position |
| `use_sim_time` | `true` | Use simulation clock |
| `enable_pickup` | `true` | Enable pick-and-place pipeline |

Example with arguments:
```bash
ros2 launch tidybot_sim tidy.launch.py enable_pickup:=false
```

### Waypoints

Edit `WAYPOINTS` in `tidybot_sim/navigator.py` to change the navigation path:

```python
WAYPOINTS = [
    (0.5, 0.0),   # start
    (2.0, 0.0),   # room 1 center
    ...
]
```

---

## Troubleshooting

### Robot falls through the floor on spawn

The robot spawns slightly above the ground (`z=0.13`). If it still falls through, verify wheel collision geometry is correct and ground plane friction is set (`mu=0.8`).

### `/cmd_vel` topic not found

The ROS-Gazebo bridge takes 2-4 seconds to initialize. The navigator waits 5 seconds before starting. If the issue persists:
```bash
ros2 topic list | grep cmd_vel
```

### Camera or LiDAR not publishing

Ensure the Sensors system plugin appears in the world SDF **and** in the robot URDF:
```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /scan
```

### Robot does not move / stuck at waypoint

Check for LiDAR obstacle detection false positives. The arm links can sometimes intrude into the forward sensing cone. Verify with:
```bash
ros2 topic echo /scan --field ranges
```

### Build fails: package not found

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-xacro
rosdep install --from-paths src --ignore-src -r -y
```

### Gazebo crashes with GPU error

Try software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch tidybot_sim tidy.launch.py
```

---

## Dependencies

### ROS 2 Packages

| Package | Purpose |
|---------|---------|
| `ros-humble-ros-gz` | Gazebo integration meta-package |
| `ros-humble-ros-gz-sim` | Gazebo simulation launch tools |
| `ros-humble-ros-gz-bridge` | ROS 2 ↔ Gazebo topic bridge |
| `ros-humble-xacro` | URDF macro processor |
| `ros-humble-robot-state-publisher` | TF tree from URDF |
| `ros-humble-joint-state-publisher` | Joint state broadcasting |

### Python Packages

See `requirements.txt`. All dependencies are standard ROS 2 Python libraries (`rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`).

---

## Deliverables Checklist

- [x] Valid ROS 2 workspace with `tidybot_sim` package
- [x] URDF robot model with all required components
- [x] Gazebo home world with two rooms, furniture, objects, collection box
- [x] Autonomous navigation (single launch command)
- [x] Sensor data publishing (camera + LiDAR)
- [x] Measurable output (CSV odometry log + terminal summary)
- [x] Runs under 5 minutes simulation time
- [x] Pick-and-place pipeline (bonus)
- [x] `requirements.txt`
- [x] `docs/APPROACH.md`
- [ ] Demo video link _(to be added)_

---

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Ignition Gazebo Documentation](https://gazebosim.org/docs)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [ros_gz_bridge](https://github.com/gazebosim/ros_gz)

No external URDF models were used as a base. All robot geometry is custom-designed to specification.
