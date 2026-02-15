# Architecture Overview

This workspace is structured around two final execution modes:

- Simulation pipeline (`fixed_path_control.launch`)
- Real vehicle pipeline (`real_vehicle_control.launch`)

## End-to-end flow

1. `racecar_cone_detection_lidar`
   - Input: `/scan`
   - Output: `/perception/lidar/cones`
2. `racecar_track_geometry`
   - Input: `/perception/lidar/cones`
   - Output: `/estimation/track/centerline` and `/planning/track/centerline_path`
3. `racecar_pure_pursuit`
   - Input: `/planning/track/centerline_path`, `/vesc/odom`
   - Output: `/planning/pure_pursuit/{delta,kappa,lookahead_distance}`
4. `racecar_pure_pursuit_control`
   - Input: `/planning/pure_pursuit/*`, `/vesc/odom`
   - Output: `/control/ackermann_cmd_mux/input/navigation`
5. Mux layer
   - Sim: `racecar_ackermann_mux` -> `/control/ackermann_cmd`
   - Real: `ackermann_cmd_mux` (high/low level) -> VESC interface

This architecture uses canonical topic naming only.

For real-vehicle deployment and pre-run validation workflow, see `docs/LABORATORY_PLAYBOOK.md`.
