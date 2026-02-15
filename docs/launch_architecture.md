# Arquitectura de Launches

## Objetivo

Estandarizar la organización de lanzadores ROS para facilitar:

- reproducibilidad experimental,
- mantenimiento incremental,
- trazabilidad académica de escenarios.

## Convención de carpetas

La estructura final usa solo lanzadores canónicos:

- `core/`: lanzadores funcionales por paquete.
- `scenarios/`: composiciones completas (simulación o coche real).
- `bringup/`: infraestructura base (control/hardware).

## Mapa canónico actual

- `racecar_cone_detection_lidar`
  - `launch/core/cone_detection.launch`
- `racecar_track_geometry`
  - `launch/core/track_estimation.launch` (incluye `midpoints`, `centerline_fit`, `centerline_to_path`, `track_viz`)
- `racecar_sim_gazebo`
  - `launch/scenarios/sim_perception_pipeline.launch`
- `racecar_sim_control`
  - `launch/bringup/low_level_control.launch`
  - `launch/scenarios/fixed_path_pipeline.launch` (simulación final)
  - `launch/scenarios/real_vehicle_pipeline.launch` (coche real final)
  - `launch/fixed_path_control.launch` (alias de simulación final)
  - `launch/real_vehicle_control.launch` (alias de real final)
- `racecar_ackermann_mux`
  - `launch/core/ackermann_mux.launch`
- `racecar_pure_pursuit`
  - `launch/core/target_selector.launch`
- `racecar_pure_pursuit_control`
  - `launch/core/controller.launch`
- `racecar_description`
  - `launch/tools/view_robot_model.launch`

## Criterios de uso

- Simulación: usar `racecar_sim_control/fixed_path_control.launch`.
- Coche real: usar `racecar_sim_control/real_vehicle_control.launch`.
- Evitar crear aliases o launches de depuración en el árbol principal.

## Operación y validación en laboratorio

- Ver `docs/LABORATORY_PLAYBOOK.md` para protocolo de despliegue, seguridad, criterios GO/NO-GO y trazabilidad experimental.
