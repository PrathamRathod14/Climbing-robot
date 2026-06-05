# Seminar Report Task Documentation

**[Insert Topic Here]**

**[Author Name 1]**  
Matriculation No. **[Matriculation No. 1]**

**[Author Name 2]**  
Matriculation No. **[Matriculation No. 2]**

**[Author Name 3]**  
Matriculation No. **[Matriculation No. 3]**

Team 09: **[Program Name]**  
Ostfalia University of Applied Sciences  
Supervisor: **[Supervisor Name]**

Salzgitter - Suderburg - Wolfenbuettel - Wolfsburg

---

## Abstract

This draft document reserves space for Task 1 and documents Task 2 of the project. Task 2 focuses on the development of a browser-based climbing robot simulation named GeckoGrip Climber. The simulation demonstrates a gecko-inspired quadruped robot that approaches a climbing wall, attaches its feet to selected holds, and climbs by moving one limb at a time while maintaining stable contact with the wall. The task combines procedural 3D modelling, route generation, gait planning, contact-state management, and interactive visualization in a Three.js environment.

---

## Contents

1. Task 1: [Reserved for Later]
2. Task 2: GeckoGrip Climber Simulation
   1. Research Problem
   2. Objectives and Scope
   3. Methodology / Approach
   4. Implementation Description
   5. Evaluation Criteria
   6. Limitations and Future Work

---

## 1. Task 1: [Reserved for Later]

This section is intentionally left as a placeholder. The final Task 1 description can be added here later without changing the structure of the document.

Suggested placeholder fields:

- **Task title:** [Insert Task 1 title]
- **Problem statement:** [Insert Task 1 problem]
- **Objective:** [Insert Task 1 objective]
- **Methodology:** [Insert Task 1 methodology]
- **Results / expected outcome:** [Insert Task 1 outcome]

---

## 2. Task 2: GeckoGrip Climber Simulation

### 2.1. Research Problem

The second task addresses the challenge of representing a climbing robot in a visually understandable and interactive simulation. Climbing robots require coordinated limb movement, reliable contact with the climbing surface, and a planning strategy that chooses reachable holds while preserving stability. A direct physical implementation would require hardware, sensors, actuators, and safety testing. Therefore, this task uses a browser-based simulation to explore the core movement logic in a controlled and repeatable environment.

The main problem investigated in this task is how a quadruped robot can transition from floor movement to wall attachment and then continue climbing by selecting suitable holds on a vertical wall.

### 2.2. Objectives and Scope

The objective of Task 2 is to design and implement a Three.js simulation that demonstrates the movement behaviour of a gecko-inspired quadruped wall climber. The simulation should make the climbing process clear to the observer by showing the robot body, four articulated limbs, climbing holds, active contacts, route progression, and status information.

The scope of this task includes:

- Creating a procedural climbing wall with multiple reachable holds.
- Modelling a Go2-inspired quadruped robot using original geometry.
- Implementing three movement phases: walking to the wall, attaching to the wall, and climbing.
- Selecting the next foot placement based on reach, direction, support width, and strain.
- Displaying simulation information such as current mode, height, gait step, active foot, and strain.
- Providing user controls for play/pause, single-step movement, reset, and wall randomization.

The task does not aim to reproduce a full physics-based hardware controller. Instead, it focuses on a visual and algorithmic demonstration of climbing behaviour.

### 2.3. Methodology / Approach

The approach follows a structured simulation pipeline:

1. Define the wall geometry and generate climbing holds.
2. Initialize the quadruped robot with four limbs attached to starting holds.
3. Move the robot from the floor toward the wall.
4. Transition each limb from floor contact to wall contact.
5. Select one limb at a time for the next climbing movement.
6. Evaluate legal holds using reach distance, forward progress, lateral distance, support width, and limb strain.
7. Animate the selected foot to the new hold and rebalance the body.
8. Update the camera, wall highlights, and dashboard values continuously.

This method separates visual rendering from movement planning so that the climbing behaviour can be inspected step by step.

### 2.4. Implementation Description

Task 2 is implemented as a Vite and TypeScript application using Three.js for real-time 3D rendering. The main simulation logic is contained in `src/main.ts`, supported by renderer, camera, controls, and styling modules.

The climbing wall is generated procedurally. Holds are arranged in rows and columns, with small variations introduced to make each wall layout less uniform. A route line provides a weak visual guide through the hold field. Each hold stores its mesh, 3D position, index, and contact state.

The robot is built from simple 3D primitives instead of relying on a proprietary robot mesh. It contains a central body, a sensor-like front module, four articulated limbs, hip and knee joints, and green contact feet. Each limb maintains its current hold, start position, target position, and side of the body.

The simulation uses three major phases:

- **Floor walk:** the robot moves across the floor toward the base of the wall.
- **Attach:** the body rotates toward the wall and the limbs move from floor positions to the first wall holds.
- **Climb:** the planner repeatedly selects a legal next hold and moves one limb at a time.

During climbing, the planner checks whether a candidate hold is reachable and whether it supports forward progress without stretching the limb beyond the configured limits. The body position is then updated from the current contact points, allowing the robot to appear balanced on the wall. Active holds are highlighted, and the dashboard reports height, gait step, active foot, and strain.

### 2.5. Evaluation Criteria

The task can be evaluated using both qualitative and functional criteria:

- **Visual correctness:** the wall, holds, robot body, limbs, and active contacts are clearly visible.
- **Phase continuity:** the robot transitions smoothly from floor walking to wall attachment and climbing.
- **Contact logic:** each limb remains associated with a hold during climbing.
- **Reach constraint:** selected holds remain within the configured leg-length limits.
- **Stability impression:** the body remains near the support center created by the four contact feet.
- **Interactivity:** play/pause, step, reset, and wall randomization controls respond as expected.
- **Reproducibility:** the simulation can be run locally with `npm install` and `npm run dev`.

### 2.6. Limitations and Future Work

The current implementation is a visual simulation rather than a full rigid-body physics model. It does not simulate friction, motor torque, sensor noise, collision forces, or adhesive foot mechanics. The climbing planner uses heuristic rules instead of a learned or optimized control policy.

Future work could extend the task by deepening the Gazebo physics model, comparing different gait strategies, recording quantitative success rates, and improving the robot model with more realistic joint limits and contact mechanics. Additional evaluation scenarios could also test different wall layouts, hold densities, and route difficulties.

---

## Bibliography

[1] Three.js Project, *Three.js Documentation*, online documentation.  
[2] Vite Project, *Vite Documentation*, online documentation.  
[3] [Insert additional robotics or climbing robot reference here].
