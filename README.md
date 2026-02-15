# tfg_racecar_ws

Workspace ROS1 (Melodic) para percepción LiDAR de conos y estimación geométrica de pista.

## Criterio de organización (estilo académico)

- `perception/*`: percepción sensorial y detecciones.
- `estimation/*`: estimación geométrica derivada de percepción.
- `planning/*`: trayectorias de referencia y elementos de planificación.
- `ground_truth/*`: referencias de simulación para evaluación.

## Pipeline vigente

1. `racecar_cone_detection_lidar`: `LaserScan -> ConeArray`
2. `racecar_track_geometry`: `ConeArray -> MidpointArray -> Centerline -> nav_msgs/Path`
3. `racecar_pure_pursuit`: selector lateral (`Ld`, `kappa`, `delta`) sobre `/planning/track/centerline_path`.
4. `racecar_pure_pursuit_control`: control longitudinal + Ackermann en `/control/ackermann_cmd_mux/input/navigation`.
5. Visualización: `track_viz` + perfiles RViz.

## Organización de launches

La estructura final se centra en `core/`, `bringup/` y `scenarios/` canónicos.
Se han retirado launches de pruebas/compatibilidad histórica para simplificar operación.

## Convención de topics principales

- Detección LiDAR:
  - `/perception/lidar/cones`
  - `/perception/lidar/cones/markers`
  - `/perception/lidar/cones/debug`
- Geometría estimada:
  - `/estimation/track/midpoints`
  - `/estimation/track/midpoints/markers`
  - `/estimation/track/centerline`
  - `/planning/track/centerline_path`
  - `/estimation/track/metrics`
  - `/estimation/track/centerline/debug`
- Control:
  - `/planning/pure_pursuit/delta`
  - `/planning/pure_pursuit/kappa`
  - `/planning/pure_pursuit/lookahead_distance`
  - `/control/ackermann_cmd_mux/input/navigation`
  - `/control/ackermann_cmd`
- Ground truth (simulación):
  - `/ground_truth/track/cones`
  - `/ground_truth/track/centerline`

## Ejecución recomendada (un solo launch)

Para levantar simulación + percepción + estimación + TF + RViz en un único comando:

`roslaunch racecar_sim_control fixed_path_control.launch`

## Ejecución en coche real (patrón Alba)

Se integra el flujo físico de `tfg_alba_ws` dentro de este workspace:

- `racecar` (launch/config de plataforma real)
- `ackermann_cmd_mux` (mux high/low level)
- `vesc/*` (driver + ackermann + odometría)
- `razor_imu_9dof` (IMU)

Launch unificado para físico + percepción + estimación + control:

`roslaunch racecar_sim_control real_vehicle_control.launch`

Notas:

- Este launch usa `racecar-v2-teleop.launch.xml` para bringup físico.
- El control longitudinal usa `pure_pursuit_control_real.yaml`, que publica en
  `/control/ackermann_cmd_mux/input/navigation`.
- Se mantiene el perfil de simulación sin cambios (`fixed_path_control.launch`).

Dependencias ROS extra para el stack físico:

`ros-melodic-serial` `ros-melodic-urg-node` `ros-melodic-joy` `ros-melodic-joy-teleop`

## Launches finales

- Simulación final:
  - `roslaunch racecar_sim_control fixed_path_control.launch`
- Coche real final:
  - `roslaunch racecar_sim_control real_vehicle_control.launch`

## Operación en laboratorio

Para la puesta en marcha en Jetson TX2 y validación paso a paso en coche real:

- `docs/LABORATORY_PLAYBOOK.md` (checklists GO/NO-GO, seguridad, bags y troubleshooting)
