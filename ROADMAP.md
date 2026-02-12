# Project Roadmap

This roadmap is my phased development plan for the autonomous aerial robotics stack. 

Hopefully, each phase produces clear, portfolio-ready deliverables and builds toward a full perception–planning–control pipeline.

---

# Phase 0 — Repository Bootstrapping

**Goal:** Establish clean repository foundations and documentation.

### Tasks

* Initialize GitHub repository.
* Create ROS 2 workspace structure.
* Add PX4 SITL submodule or integration instructions.
* Add initial README and roadmap.

### Deliverables

* Public repository with foundational structure.
* Basic CI for formatting/linting.

---

# Phase 1 — PX4 + ROS 2 Integration (SITL)

**Goal:** Achieve a functioning PX4 SITL simulation bridged to ROS 2.

### Tasks

* Install PX4 dev environment.
* Set up MAVROS2 or microRTPS bridge.
* Spawn drone model in Gazebo/Ignition.
* Implement ROS 2 node for basic commands (arm, takeoff, land).

### Deliverables

* Launch file bringing up PX4 + ROS 2.
* Demo video: takeoff → hover → land.

---

# Phase 2 — Perception Pipeline (Monocular)

**Goal:** Build a modern perception pipeline replacing the original master’s thesis work.

### Modules

* Monocular depth estimation (MiDaS/Leres/ZoeDepth).
* Visual Odometry/SLAM (ORB-SLAM3 or VINS-Fusion).
* Object detection (YOLOv8/YOLOv9).

### Deliverables

* Independent ROS 2 perception nodes.
* ROS topics: `/depth_map`, `/pose`, `/detections`.
* Unit tests and sample recordings.

---

# Phase 3 — Mapping + Planning

**Goal:** Enable autonomous navigation with planning and obstacle avoidance.

### Modules

* Global planner: RRT*, A*.
* Local planner: DWA or reactive avoidance.
* Basic trajectory generation.

### Deliverables

* Planner nodes publishing to `/trajectory_setpoint`.
* Simulation demo: navigate to waypoint with obstacles.

---

# Phase 4 — Mission Logic

**Goal:** Build high-level autonomy behaviors.

### Behaviors

* Route execution.
* Autonomous return-to-home.

### Deliverables

* Mission node with configurable strategies.

---

# Phase 5 — Simulation Enhancements

**Goal:** Improve realism and robustness.

### Tasks

* Add environmental variations (lighting, weather).
* Improve camera plugin models.

### Deliverables

* Enhanced simulation world.
* Benchmark results across conditions.



This is a flexible roadmap, designed to scale in complexity as the project evolves.

