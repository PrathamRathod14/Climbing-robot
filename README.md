# GeckoGrip Climbing Robot Simulation

GeckoGrip is a climbing-robot simulation project that demonstrates a Unitree Go2-style quadruped climbing a vertical wall. The project includes a Gazebo and RViz workflow for the main Task 3 simulation, plus a browser-based Three.js preview for quick visualization.

## Demo Video

[Watch the climbing robot demo video](https://drive.google.com/file/d/1YdWoWxYY0DKg8GgpMOOHjo5t18DwjKyL/view)

## Project Overview

The simulation focuses on a wall-climbing task where the robot detects yellow target blocks, places one leg at a time, shifts its body upward, and repeats the gait sequence. As the robot climbs, the simulated gravity/load value increases to make the climb more difficult. When the load becomes too high and stability is lost, the robot falls, recovers on the ground, returns to the starting position, and begins the climb again.

RViz is used alongside Gazebo to visualize robot state, contact points, target markers, gravity/load information, support indicators, and status text.

## Key Features

- Gazebo wall-climbing world with ground, wall, yellow climbing blocks, and robot model.
- Unitree Go2-style quadruped climbing sequence with repeated leg placement.
- Yellow-block target selection for reachable footholds/handholds.
- Increasing gravity/load visualization during the climb.
- Fall, recovery, restart, and replay behaviour after loss of stability.
- RViz bridge for synchronized joint states, transforms, contact markers, gravity arrows, support markers, and text overlays.
- Three.js browser preview for inspecting the climbing concept without launching Gazebo.

## Repository Structure

```text
.
├── gazebo/
│   ├── geckogrip.world
│   ├── geckogrip_climb_animator.cc
│   ├── gazebo_rviz_bridge.py
│   ├── gazebo_rviz.launch.py
│   ├── compile.sh
│   ├── launch_demo.sh
│   └── start_gazebo_climb_demo.ps1
├── public/
│   ├── route1.txt
│   ├── route2.txt
│   └── route3.txt
├── src/
│   ├── main.ts
│   ├── wall.ts
│   ├── kinematics.ts
│   └── core/
├── task-documentation.md
├── package.json
└── README.md
```

## Gazebo and RViz Simulation

The Gazebo workflow is the main technical simulation. It compiles the climbing animator plugin, loads the wall environment, starts the climbing cycle, and launches RViz visualization.

### Windows PowerShell

```powershell
.\gazebo\start_gazebo_climb_demo.ps1
```

### Linux / WSL

```sh
cd gazebo
./compile.sh
./launch_demo.sh
```

## Web Preview

The web preview is built with Vite, TypeScript, and Three.js. It provides a quick browser visualization of the climbing robot concept, wall holds, gait progression, and interaction controls.

```sh
npm install
npm run dev
```

Then open the local URL printed by Vite.

## Build

```sh
npm run build
```

## Technical Flow

1. Gazebo loads the wall, floor, yellow climbing blocks, robot model, lighting, and climbing plugin.
2. The active leg searches for the nearest reachable yellow block.
3. The selected leg moves toward the target block while the other contact points act as support.
4. The robot body shifts upward after each successful leg placement.
5. The gravity/load value increases as the climb progresses.
6. RViz publishes synchronized robot state, target/contact markers, support information, gravity direction, and load text.
7. If the robot loses stability under the increased load, it falls to the ground.
8. The recovery logic returns the robot to the lower wall section and restarts the climbing cycle.

## Main Implementation Files

- `gazebo/geckogrip.world` defines the Gazebo environment, wall, floor, yellow blocks, lighting, and plugin attachment.
- `gazebo/geckogrip_climb_animator.cc` contains the Gazebo-side climbing cycle, gait timing, target selection, gravity/load ramp, fall logic, recovery, and restart behaviour.
- `gazebo/gazebo_rviz_bridge.py` publishes joint states, transforms, contact markers, target markers, gravity/load overlays, and support visualization to RViz.
- `gazebo/start_gazebo_climb_demo.ps1` provides the repeatable Windows launch workflow.
- `src/main.ts` contains the Three.js browser simulation logic.

## Current Limitations

This project is a task-oriented simulation rather than a complete real-world controller. The gravity/load ramp, contact behaviour, and recovery sequence are designed to make the climbing behaviour understandable and repeatable. They do not fully model real adhesive feet, friction limits, actuator torque, sensor noise, or rigid-body contact dynamics.

## Future Improvements

- Add more detailed foot-block contact physics, including friction and grip thresholds.
- Replace scripted recovery with controller-based balancing and recovery.
- Record quantitative metrics such as climb time, fall height, successful placements, and maximum load before failure.
- Extend target detection with camera or depth-sensor input.
- Compare different gait strategies and wall layouts.
