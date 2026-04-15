# AI-Driven Digital Twin — Camera Agent Target Tracking

![Digital Twin Overview](media/warehouse_digital_twin.gif)


## 🚀 Project Overview

This project demonstrates an **AI-driven camera agent operating inside a Digital Twin environment** performing real-time scanning, target acquisition, and lock-on using bounding box detection and spatial reasoning built in NVIDIA Omniverse + Isaac Sim.

The agent:
- Starts from a fixed overview position
- Navigates through the environment using waypoint logic
- Actively scans for a target (red warning light)
- Reacquires the target if lost
- Performs a controlled fly-in
- Executes final visual alignment using bounding-box perception

This is a **full perception → decision → action loop**, representing a foundational architecture for autonomous systems in digital twins.

---

## 🎯 Key Achievement

> The Camera Agent successfully detects, navigates to, and visually locks onto a real target in a complex 3D environment — without requiring a predefined starting distance.

📹 **Proof (Core Result):**  
`media/Camera_Agent_Track_To_RED_Light_Target.mp4`

---

## 🧠 System Architecture

### 1. Perception Layer
- Synthetic camera using Replicator
- Semantic labeling (`warning_light`)
- Bounding box detection (`bounding_box_2d_tight`)
- Multi-prim merging into a unified target

### 2. Decision Layer (State Machine)

The agent operates using a structured state system:

scan_overview → move_to_overview → scan_investigate →
move_to_investigate → approach_target → inspect_hold



Each state is responsible for a specific behavior:
- Exploration
- Target acquisition
- Navigation
- Final alignment

---

### 3. Control Layer

- Yaw-based steering
- Forward vector projection for movement
- Visual servoing using bounding box center error
- Dead-zone stabilization to prevent oscillation

---

## 🔍 Visual Servoing (Core Innovation)

At close range, the agent transitions from navigation to **vision-driven alignment**:

```python
bbox_error = bbox_center_x - image_center_x
yaw += k * bbox_error

This allows:

Smooth target centering
Stable lock-on behavior
Real-time correction without oscillation
🏗️ Environment

The Digital Twin represents a warehouse / server environment with:

Conveyor systems
Structural framing
Server racks
Workstations
Target: Red warning light

```

## 📸 Highlights

These renders showcase the Digital Twin environment from multiple functional perspectives, highlighting system layout, asset organization, and the operational context in which the Camera Agent performs perception and target tracking.

### 1. Full System Overview
A top-down perspective of the entire warehouse digital twin, showing conveyor routing, workstation zones, storage areas, and the central server enclosure where the target is located. This establishes the global spatial context the agent must navigate.

![Hero System Overview](images/digital_twin_warehouse_01.png)

### 2. Storage & Inventory Zone
A focused view of the storage racks and boxed inventory, demonstrating structured asset placement and repeatable geometry—key for testing perception consistency and spatial reasoning in cluttered environments.

![Inventory Zone](images/digital_twin_warehouse_02.png)

### 3. Workstation Integration
A closer look at operator workstations within the environment, illustrating how human-centric elements are integrated into the digital twin alongside automated systems, reinforcing real-world applicability.

![Workstation Detail](images/digital_twin_warehouse_03.png)

### 4. Conveyor System Detail
A detailed shot of the conveyor junction and transfer mechanism, highlighting mechanical layout and flow design—important for understanding environmental complexity and potential occlusion scenarios.

![Conveyor Detail](images/digital_twin_warehouse_04.png)

### 5. Target Zone (Server Core)
An overhead view of the central server enclosure where the warning light target resides, emphasizing structural occlusion, lighting contrast, and the challenge of reliable visual detection within dense geometry.

![Server Core](images/digital_twin_warehouse_05.png)

--- 


## 🧪 Technical Stack

NVIDIA Omniverse
Isaac Sim 4.2 (containerized)
OpenUSD (scene composition & transforms)
Python (control + perception loop)
Replicator (synthetic data + annotations)



## ⚙️ Key Features

Fixed world-space start pose
Waypoint-based navigation
Dynamic target acquisition
Occlusion handling
Reacquisition via scan behavior
Stabilized stopping logic
Bounding-box driven alignment



## 🧱 Project Structure
project_07_ai_driven_digital_twin_system/
├── images/
├── media/
├── isaac/
│   └── agent_perception_*.py
├── models/
├── usd/
├── output/
└── docs/



## 🧭 What This Demonstrates

This project showcases:

Real-time perception in simulation
Digital Twin interaction logic
Autonomous agent behavior design
OpenUSD-based scene reasoning
Practical robotics-style control systems



## 🔮 Next Steps

Multi-agent coordination
Depth-based navigation (not just vision)
Full 6-DOF camera control (pitch + yaw)
Integration with learning-based models (DQN / policy learning)
Cosmos pipeline integration (sim → real transfer)



## 🏁 Summary

This is not just a simulation.

It is a functional AI agent operating inside a Digital Twin, capable of:

Perceiving its environment
Making decisions
Acting with purpose
Achieving a defined objective


## 👤 Author

Dartayous Hunter
Digital Twin Engineer (OpenUSD | Omniverse | AI Systems)