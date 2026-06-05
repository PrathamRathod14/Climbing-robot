# GeckoGrip Gazebo Wall Climber

Gazebo-first wall-climbing robot demo with RViz gravity/load visualization.

The robot climbs to about 65% of the wall, the simulated gravity/load visualization ramps up as it gains height, and then the robot releases and falls.

## Gazebo + RViz

```sh
cd gazebo
./compile.sh
./launch_demo.sh
```

On Windows/PowerShell:

```powershell
.\gazebo\start_gazebo_climb_demo.ps1
```

## Web Preview

The Three.js preview is still available for quick browser visualization.

```sh
npm install
npm run dev
```

## Build

```sh
npm run build
```
