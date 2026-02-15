# Runbook

## Prerequisites

- ROS Melodic installed
- Dependencies for real stack:
  - `ros-melodic-serial`
  - `ros-melodic-urg-node`
  - `ros-melodic-joy`
  - `ros-melodic-joy-teleop`

## Build

```bash
cd /ws/ros1/melodic/tfg_mario_ws/tfg_racecar_ws
catkin_make
source devel/setup.bash
```

## Simulation (final)

```bash
roslaunch racecar_sim_control fixed_path_control.launch
```

## Real vehicle (final)

```bash
roslaunch racecar_sim_control real_vehicle_control.launch
```

## Laboratory operation (recommended)

Follow the full deployment and validation checklist in:

- `docs/LABORATORY_PLAYBOOK.md`

## Useful checks

```bash
rostopic list | rg "/control/|/planning/pure_pursuit|/perception/|/estimation/"
rostopic echo /control/ackermann_cmd
```
