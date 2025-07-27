# Autonomous Robot Localization and Navigation System

This project was developed for the **ECE4180 Final Competition** at Cornell University. Our robot autonomously localized itself from an unknown starting point, planned collision-free paths using global planning algorithms, and navigated through a mapped environment while detecting obstacles in real time.

---

## Project Overview

- Combined **Particle Filter (PF)** for localization and **RRT + Dijkstra** for path planning.
- Localized from an unknown pose using a 360° sensor sweep and initialized 700 particles.
- Dynamically updated the map with optional walls detected via bump sensing.
- Planned paths around stay-away zones and optimized traversal of standard and extra credit waypoints.
- Developed robust real-time visualization tools for debugging and performance tuning.

---

## Key Features

- **Accurate Localization**  
  PF tracked robot pose with ±0.1 m accuracy during turns and linear motion.

- **Hybrid Navigation Strategy**  
  RRT generated global paths; Dijkstra selected the optimal route to each waypoint.

- **Modular Real-Time Control**  
  Robot continuously re-planned paths and updated belief states during runtime.

- **Obstacle Detection & Map Update**  
  Optional walls were added to the map dynamically based on bump sensor readings.

- **Visualization Tools**  
  Live plots of robot trajectory, particles, RRT roadmaps, and stay-away zones.

---

## Ivan Huang's Contributions

- Proposed and implemented the full system architecture: PF + RRT + Dijkstra.
- Built major modules:
  - `initialPF` — 700-particle initialization with 360° spin.
  - `navigPF` — path planning, state management, and navigation.
  - `goToWalls` — mapping optional walls via bump detection.
- Tuned PF parameters and RRT settings for smooth navigation.
- Integrated all modules into a cohesive control system.
- Designed and implemented real-time visualization tools.
- Spent 25+ hours on testing and debugging for reliability.

---

## Skills & Experience Gained

### Embedded System Development
- Real-time control with depth, odometry, and bump sensors.
- Designed control logic within tight speed and timing constraints.

### Model Building & Simulation
- Built and tuned a probabilistic PF model for localization.
- Used simulation to verify logic and test navigation performance.

### Algorithm Implementation
- Implemented RRT and Dijkstra from scratch for onboard motion planning.
- Designed logic to avoid collisions and re-plan as needed.

### System Integration & Debugging
- Developed tools to visualize and debug particle filters and map data.
- Performed full-system integration and live testing under real constraints.

---

## Competition Result

While the robot performed robustly and accurately in simulation and physical tests, strict speed and time limits in the final run prevented reaching all waypoints. Nevertheless, the system showed strong localization, navigation, and adaptability, making it an excellent showcase of embedded robotics and systems engineering.

---

## Example Visualizations 
![Robot Simulation](images/simulation_result.png)

---

## Repository Structure 

