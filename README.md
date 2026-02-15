# Autonomous Aerial Robotics Stack

This repository documents an in-progress, simulation-first exploration into integrating **PX4** and **ROS 2**. 

My goal is to learn, experiment, and gradually build towards a modular **perception → planning → control** pipeline. **PX4**, **ROS 2**, and a modern **perception → planning → control** pipeline. 

I am designing this system for research, portfolio development. 
Hope it too helps you in your journey.


Naturally, I am trying to learn and move fast. So I will leverage the hell out of any AI tools I can find when it comes to writing the actual code. 
Goal is to learn. And we gotta move quick.


## Overview

The architecture is designed around a clear separation of concerns:

* **PX4 Autopilot** for low-level flight control, state estimation, and safety.
* **ROS 2** for high-level autonomy: perception, SLAM, planning, mission logic.
*  [pending decision] **MAVROS2** for PX4↔ROS 2 communication.
*  [pending decision] **Gazebo/Ignition** for simulation and testing.

This project aims to rebuild and modernize a monocular perception pipeline (originally part of my robotics master’s thesis) using current computer vision and ML tools, while openly sharing the learning process with the community.

## Planned Features

* PX4 SITL integration with ROS 2
* Modular perception pipeline:

  * Monocular depth estimation
  * Visual SLAM / VO
  * Object detection
  * Optional semantic segmentation
* Path planning:

  * RRT*, A*, or DWA-based local planning
* Mission logic:

  * Parimeter Patrol
  * Tracking
  * Autonomous navigation tasks
* Simulation environment with camera plugins
* Clean ROS 2 workspace and launch system

## Repository Structure

```
/px4_ws          # PX4 SITL and bridging
/ros2_ws         # ROS2 workspace
  /src
    perception/  # Depth, VO/SLAM, detection
    planning/    # Global + local planners
    missions/    # Mission logic
    utils/       # Shared tools
/docs            # Diagrams, design docs
/scripts         # Helper scripts
```

## Getting Started

1. Make sure you install the [hello sky simulation environment](https://docs.px4.io/main/en/dev_setup/dev_env_mac) from PX4



1. Install required dependencies (ROS 2, PX4 SITL, Gazebo/Ignition).
2. Build PX4 and ROS 2 workspaces.
3. Launch simulation with:

```
ros2 launch autonomous_stack bringup.launch.py
```

4. Run perception or planning modules individually for testing.

## Goals

* Build a modern, open-source autonomy stack.
* Produce high-quality engineering documentation.
* Explore commercial directions (e.g., aerial surveillance).

See `ROADMAP.md` for detailed milestones and development phases.

