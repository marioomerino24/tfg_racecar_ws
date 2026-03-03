# Convencion de Nombrado ROS

## Reglas generales

- Idioma tecnico: ingles para identificadores ROS (topics, frames, nodos).
- Estilo: `snake_case` para topics, nodos y parametros.
- Namespaces por dominio funcional.

## Dominios de topics

| Prefijo | Contenido | Ejemplo |
|---------|-----------|---------|
| `/perception/` | Salidas de sensores procesados | `/perception/lidar/cones` |
| `/estimation/` | Estimaciones geometricas derivadas | `/estimation/track/midpoints` |
| `/planning/` | Trayectorias y comandos de referencia | `/planning/track/centerline_path` |
| `/control/` | Comandos hacia actuadores | `/control/ackermann_cmd` |
| `/ground_truth/` | Referencias de simulacion (solo Gazebo) | `/ground_truth/track/cones` |

## Tabla completa de topics canonicos

### Percepcion

| Topic | Tipo | Productor |
|-------|------|-----------|
| `/perception/lidar/cones` | `ConeArray` | `cone_detector_node` |
| `/perception/lidar/cones/markers` | `MarkerArray` | `cone_detector_node` |
| `/perception/lidar/cones/debug` | texto serial | `cone_detector_node` |

### Estimacion

| Topic | Tipo | Productor |
|-------|------|-----------|
| `/estimation/track/midpoints` | `MidpointArray` | `midpoints_node` |
| `/estimation/track/midpoints/markers` | `Marker` | `track_viz` |
| `/estimation/track/centerline` | `Centerline` | `centerline_fit_node` |
| `/estimation/track/centerline/debug` | texto serial | `centerline_fit_node` |
| `/estimation/track/metrics` | `TrackMetrics` | `midpoints_node` |

### Planificacion

| Topic | Tipo | Productor |
|-------|------|-----------|
| `/planning/track/centerline_path` | `nav_msgs/Path` | `centerline_to_path_node` |
| `/planning/pure_pursuit/delta` | `std_msgs/Float64` | `pure_pursuit_target_selector` |
| `/planning/pure_pursuit/kappa` | `std_msgs/Float64` | `pure_pursuit_target_selector` |
| `/planning/pure_pursuit/lookahead_distance` | `std_msgs/Float64` | `pure_pursuit_target_selector` |

### Control

| Topic | Tipo | Productor |
|-------|------|-----------|
| `/control/ackermann_cmd_mux/input/navigation` | `AckermannDriveStamped` | `pure_pursuit_controller` |
| `/control/ackermann_cmd_mux/input/teleop` | `AckermannDriveStamped` | `joy_teleop` |
| `/control/ackermann_cmd_mux/input/safety` | `AckermannDriveStamped` | (e-stop) |
| `/control/ackermann_cmd` | `AckermannDriveStamped` | `ackermann_mux` |

## Frames TF

| Frame | Tipo | Publicador |
|-------|------|------------|
| `map` -> `odom` | estatico | `static_transform_publisher` |
| `odom` -> `base_link` | dinamico | `vesc_to_odom` / `gazebo_odometry` |
| `base_link` -> `laser` | estatico | `static_transforms.launch.xml` |
| `base_link` -> `base_imu_link` | estatico | `static_transforms.launch.xml` (solo real) |

## Estilo de parametros

Preferir claves jerarquicas en YAML:

```yaml
# Bien
input:
  scan_topic: "/scan"
output:
  cones_topic: "/perception/lidar/cones"

# Evitar
scan_topic_input: "/scan"
cones_topic_output: "/perception/lidar/cones"
```

## Politica de migracion

Los nombres legacy han sido eliminados de la version final.
Todos los nodos y launches usan exclusivamente los topics canonicos listados arriba.
