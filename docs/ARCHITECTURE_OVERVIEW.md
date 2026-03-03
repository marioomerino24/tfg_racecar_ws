# Arquitectura del Sistema

Este workspace implementa un pipeline de conduccion autonoma para un racecar 1/10 basado en deteccion LiDAR de conos.

## Modos de ejecucion

| Modo | Launch | Plataforma |
|------|--------|------------|
| Simulacion | `roslaunch racecar_sim_control fixed_path_control.launch` | Gazebo + ros_control |
| Vehiculo real | `roslaunch racecar_sim_control real_vehicle_control.launch` | Jetson TX2 + VESC + Hokuyo |

Ambos modos comparten el mismo pipeline de percepcion, estimacion y control.
La unica diferencia es el bringup de hardware (simulado vs fisico).

## Pipeline end-to-end

```
 Sensor           Percepcion           Estimacion             Planificacion          Control
 ------           ----------           ----------             -------------          -------

 /scan ------> cone_detector ------> midpoints_node ------> centerline_fit ------> pp_target_selector
(LaserScan)    (ConeArray)          (MidpointArray)         (Centerline)           (delta, kappa, Ld)
                                         |                       |                       |
                                         v                       v                       v
                                    TrackMetrics          centerline_to_path       pp_controller
                                                          (nav_msgs/Path)     (AckermannDriveStamped)
                                                                                        |
                                                                                        v
                                                                                   ackermann_mux
                                                                                        |
                                                                                        v
                                                                                  /control/ackermann_cmd
                                                                                   (VESC / Gazebo)
```

## Paquetes por etapa

### 1. Percepcion: racecar_cone_detection_lidar

Detecta conos a partir del scan LiDAR usando clustering euclidiano y ajuste circular.

- **Entrada:** `/scan` (`sensor_msgs/LaserScan`)
- **Salida:** `/perception/lidar/cones` (`ConeArray`)
- **Algoritmo:** Clustering euclidiano + Hyper circle fit (Al-Sharadqah & Chernov) con fallback a centroide.
- **Config:** `racecar_cone_detection_lidar/config/detector.yaml`

### 2. Estimacion: racecar_track_geometry

Convierte los conos detectados en una representacion geometrica de la pista.

**midpoints_node:**
- **Entrada:** `/perception/lidar/cones` (`ConeArray`)
- **Salida:** `/estimation/track/midpoints` (`MidpointArray`), `/estimation/track/metrics` (`TrackMetrics`)
- **Algoritmo:** Binning longitudinal + estadistica robusta (mediana/MAD) + latching de estabilidad.

**centerline_fit_node:**
- **Entrada:** `/estimation/track/midpoints` (`MidpointArray`)
- **Salida:** `/estimation/track/centerline` (`Centerline`)
- **Algoritmo:** Spline cubica natural + algoritmo de Thomas O(n) + remuestreo uniforme.
- **Config:** `racecar_track_geometry/config/centerline.yaml`

**centerline_to_path_node:**
- **Entrada:** `/estimation/track/centerline` (`Centerline`)
- **Salida:** `/planning/track/centerline_path` (`nav_msgs/Path`)

**track_viz:**
- Visualizacion unificada de todos los elementos en RViz.
- **Config:** `racecar_track_geometry/config/track_viz.yaml`

### 3. Planificacion: racecar_pure_pursuit

Selector de target para el controlador Pure Pursuit con lookahead adaptativo.

- **Entrada:** `/planning/track/centerline_path`, `/vesc/odom`
- **Salidas:**
  - `/planning/pure_pursuit/delta` (angulo de direccion, `Float64` [rad])
  - `/planning/pure_pursuit/kappa` (curvatura local, `Float64` [1/m])
  - `/planning/pure_pursuit/lookahead_distance` (distancia Ld, `Float64` [m])
- **Modos de lookahead:** `constant`, `speed`, `curvature`, `hybrid`
- **Config:** `racecar_pure_pursuit/config/pure_pursuit.yaml`

### 4. Control: racecar_pure_pursuit_control

Control longitudinal con limites de seguridad multinivel.

- **Entrada:** `/planning/pure_pursuit/{delta,kappa,lookahead_distance}`, `/vesc/odom`
- **Salida:** `/control/ackermann_cmd_mux/input/navigation` (`AckermannDriveStamped`)
- **Limites de velocidad:**
  1. Curvatura: `v_max = sqrt(a_lat_max / (|kappa| + eps))`
  2. Time-headway: `v_h = Ld / T_h`
  3. Absoluto: `v_max` (config)
- **Seguridad:** Watchdog con timeout por cada entrada. Si caduca, publica `stop`.
- **Config:** `racecar_pure_pursuit_control/config/pure_pursuit_control.yaml`

### 5. Multiplexor

Seleccion por prioridad del comando Ackermann final.

| Plataforma | Paquete | Prioridades |
|------------|---------|-------------|
| Simulacion | `racecar_ackermann_mux` (Python) | e_stop(100) > teleop(50) > navigation(10) |
| Real | `ackermann_cmd_mux` (C++) + mux high/low level | teleop(10) > safety(5) > navigation(0) |

### 6. Hardware (solo vehiculo real)

| Paquete | Funcion |
|---------|---------|
| `vesc/vesc_driver` | Comunicacion serie con ESC (C++) |
| `vesc/vesc_ackermann` | Conversion Ackermann <-> VESC + odometria |
| `razor_imu_9dof` | Driver IMU 9 DOF por serie (C++) |
| `racecar` | Launch/config de bringup fisico |

## Configuracion centralizada

Las constantes del vehiculo se definen una sola vez:

```
racecar_sim_control/config/
  vehicle/racecar-v2.yaml    <-- wheelbase, delta_max (fuente unica)
  common.yaml                <-- topics y frames compartidos
  overrides/sim.yaml         <-- diferencias de simulacion
  overrides/real.yaml        <-- diferencias de vehiculo real
```

Los nodos de control resuelven `wheelbase_L` y `delta_max_deg` desde `/vehicle/` namespace
del parameter server. Ver `config/vehicle/racecar-v2.yaml`.

## Frames TF

```
map (estatico)
 |
 v
odom (dinamico: vesc_to_odom o gazebo_odometry)
 |
 v
base_link
 |
 +---> laser (estatico)
 +---> base_imu_link (estatico, solo real)
```

La cadena critica es `odom -> base_link -> laser`. Si falta, el pipeline de
percepcion no puede transformar detecciones y el control se detiene.

## Documentacion relacionada

- [MSG_API.md](MSG_API.md) — referencia de mensajes custom
- [LABORATORY_PLAYBOOK.md](LABORATORY_PLAYBOOK.md) — protocolo de despliegue en vehiculo real
- [launch_architecture.md](launch_architecture.md) — convencion de launch files
- [ROS_NAMING_STANDARD.md](ROS_NAMING_STANDARD.md) — convencion de nombrado de topics
- [RUNBOOK.md](RUNBOOK.md) — referencia rapida de ejecucion
