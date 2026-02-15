# racecar_cone_detection_lidar

Deteccion de conos basada en LiDAR para el pipeline de geometria de pista.

## Nodo principal

- Ejecutable recomendado: `cone_detector_node.py`
- Launch recomendado: `launch/core/cone_detection.launch`

Entrada:
- `sensor_msgs/LaserScan` en `~input/scan_topic` (default: `/scan`)

Salida:
- `racecar_cone_msgs/ConeArray` en `~output/cones_topic` (default: `/perception/lidar/cones`)
- `visualization_msgs/MarkerArray` en `~output/markers_topic` (default: `/perception/lidar/cones/markers`)
- `std_msgs/String` debug serial en `~debug/serial_topic` (default: `/perception/lidar/cones/debug`)

## Limpieza aplicada

- Eliminadas versiones antiguas: `scan_to_cones_prev3.py`, `scan_to_cones_prev4.py`.
- Eliminados launches legacy de alias/evaluación para dejar solo el launch canónico.
