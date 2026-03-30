## My Build vs. Drift's Build
In the simulation that I made, the room structure was more correct than that of Drift's. In the Drift's simulation, the wall separating the two rooms didn't have a gap. So the robot had no way to cross the room. Also, the starting point in Drift's simulation was a bit congested. Because of that, the robot was stuck at the initial waypoint itself. Since it was stuck at the initial waypoint, I was not able to see how well the object detection and pick-and-place behaviors worked.

- My Build : [Link](https://drive.google.com/file/d/1w3XHiQi6Qk40-G-TXegO0mvAY2g0DL9P/view?usp=sharing)
- Drift's Build : [Link](https://drive.google.com/file/d/1zGtI-lmarVDs-I95Ha3CZCHSyT318vbx/view?usp=sharing)

## Robot Model Construction
The base mobility platform was inspired by an open-source [4-wheel differential rover](https://github.com/Abdelrahman-Galal/4-wheel-differential-mobile-robot/blob/main/urdf/car-robot.urdf). Utilizing this structural baseline, alongside AI assistance (Gemini, Claude) for syntax formatting and boilerplate generation, I developed a custom URDF that strictly adheres to the physical and kinematic specifications required.

## Environment Design
The home layout was optimized for simulation performance. Structural elements and large furniture are configured as static bodies to reduce physics solver overhead. Conversely, the small target objects (cubes) are fully dynamic, complete with calculated mass, friction, and inertial properties to allow for physical interaction.

## Navigation Strategy
To guarantee reliable, collision-free traversal between rooms, I implemented a deterministic, waypoint-based navigation system. The robot follows a hardcoded sequence of spatial coordinates to execute the tidying route.

## Tradeoffs & Future Work

1. CPU vs. GPU Raycasting:
Initially, I configured the LiDAR to utilize the GPU (gpu_lidar). However, due to environment-specific rendering crashes, I transitioned to a CPU-based lidar implementation. While GPU acceleration is preferable for massive point clouds, the CPU fallback uses Bullet/Dart physics for raycasting rather than the OGRE2 render engine, providing identical 360° scan data with guaranteed stability on any hardware.

2. Pick-and-Place State Machine:
The pick-and-place pipeline is currently partially implemented. While the robot successfully detects the target objects, the physical reach constraints of the current arm design make reliable grasping inconsistent. Given more time, I would redesign the arm's kinematics for better floor clearance and refactor the state transitions to handle dynamic, on-the-move pickups.

## Debugging Log

### Bug 1: Simulation Server Segfault on Launch
- Root Cause A (Sensor Plugin Duplication): The Sensors plugin was defined in both world.sdf and the robot's URDF. During spawn, sdformat_urdf converts URDF plugins to world-level plugins, resulting in two Sensors system instances. The second instance attempted to re-register visuals already owned by the first, causing a [Visual] already exists segfault.

    - Fix: Removed the Sensors plugin from the URDF entirely, ensuring it lives exclusively as a single instance in world.sdf.

- Root Cause B (OpenGL/EGL Context Failure): The OGRE2 render engine was crashing when attempting to fire rays for the gpu_lidar or render the camera stream without a dedicated GPU context.

    - Fix: Injected SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1') into the launch file prior to the Gazebo IncludeLaunchDescription. This forces Mesa to use llvmpipe (a full software OpenGL rasterizer). While this drops rendering performance (~100ms/frame), it ensures the camera and simulation run flawlessly on machines without a dedicated GPU.

### Bug 2: Infinite PICKUP State Hang

- Symptom: The navigation state machine would occasionally enter the PICKUP state and wait indefinitely because there was no exit condition other than a successful pickup_done signal.

- Fix: Implemented a watchdog timer. Added a pickup_start timestamp and a 20-second timeout within the PICKUP state handler. If the arm controller fails or stalls, the navigator now automatically aborts the grasp and resumes the patrol.

