# racecar_track_geometry

Estimación geométrica de pista a partir de conos detectados.

## Alcance actual

- Entrada: `racecar_cone_msgs/ConeArray` (conos detectados).
- Salidas:
  - `racecar_cone_msgs/MidpointArray`
  - `racecar_cone_msgs/Centerline`
  - `racecar_cone_msgs/TrackMetrics`
  - `visualization_msgs/MarkerArray` (track_viz)

## Pipeline interno

1. `midpoints_node.py`: discretización longitudinal y estimación robusta de midpoints.
2. `centerline_fit_node.py`: ajuste spline cúbico natural y métricas.
3. `track_viz.py`: visualización y publicación de marcadores.

## Nota de limpieza

Los nodos legacy de emparejamiento L-R han sido retirados para alinear el paquete con el flujo vigente basado en midpoints directos.
