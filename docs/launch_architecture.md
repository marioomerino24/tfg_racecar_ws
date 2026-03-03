# Arquitectura de Launches

## Objetivo

Estandarizar la organizacion de lanzadores ROS para facilitar:

- reproducibilidad experimental,
- mantenimiento incremental,
- trazabilidad academica de escenarios.

## Convencion de carpetas

| Carpeta | Proposito | Ejemplo |
|---------|-----------|---------|
| `core/` | Lanzadores funcionales por paquete (un nodo o grupo logico) | `cone_detection.launch` |
| `scenarios/` | Composiciones completas (pipeline entero) | `fixed_path_pipeline.launch` |
| `bringup/` | Infraestructura base (hardware, controladores) | `low_level_control.launch` |

Los aliases de compatibilidad (`fixed_path_control.launch`, `real_vehicle_control.launch`)
estan en la raiz de `racecar_sim_control/launch/` y simplemente incluyen el scenario correspondiente.

## Mapa canonico

```
racecar_cone_detection_lidar/
  launch/core/cone_detection.launch

racecar_track_geometry/
  launch/core/track_estimation.launch    (midpoints + centerline + path + viz)

racecar_pure_pursuit/
  launch/core/target_selector.launch

racecar_pure_pursuit_control/
  launch/core/controller.launch

racecar_ackermann_mux/
  launch/core/ackermann_mux.launch

racecar_sim_gazebo/
  launch/scenarios/sim_perception_pipeline.launch

racecar_sim_control/
  launch/bringup/low_level_control.launch
  launch/scenarios/fixed_path_pipeline.launch    (simulacion final)
  launch/scenarios/real_vehicle_pipeline.launch   (vehiculo real final)
  launch/fixed_path_control.launch                (alias -> scenarios/)
  launch/real_vehicle_control.launch              (alias -> scenarios/)

racecar_description/
  launch/tools/view_robot_model.launch

racecar/
  launch/includes/common/joy_teleop.launch.xml
  launch/includes/common/joy_teleop_sim.launch.xml
  launch/includes/common/sensors.launch.xml
  launch/includes/racecar-v2-teleop.launch.xml
  launch/includes/racecar-v2/vesc.launch.xml
  launch/includes/racecar-v2/static_transforms.launch.xml
  launch/mux.launch
```

## Configuracion centralizada en launches

Ambos pipelines (sim y real) cargan las constantes del vehiculo al inicio:

```xml
<rosparam file="$(find racecar_sim_control)/config/vehicle/racecar-v2.yaml"
          command="load" ns="vehicle"/>
```

Esto hace disponible `/vehicle/wheelbase_m`, `/vehicle/delta_max_deg`, etc. en el
parameter server para todos los nodos del pipeline.

## Criterios de uso

- **Simulacion:** `roslaunch racecar_sim_control fixed_path_control.launch`
- **Vehiculo real:** `roslaunch racecar_sim_control real_vehicle_control.launch`
- No crear aliases ni launches de depuracion en el arbol principal.
- Los parametros de algoritmo van en YAML (`config/`), no hardcodeados en el launch.

## Patron de paso de config a nodos core

Cada launch `core/` recibe la ruta al YAML por argumento:

```xml
<launch>
  <arg name="config" default="$(find paquete)/config/default.yaml"/>
  <rosparam file="$(arg config)" command="load"/>
  <node pkg="paquete" type="nodo.py" name="nodo" output="screen"/>
</launch>
```

Esto permite que los `scenarios/` sustituyan el YAML sin modificar el `core/`.
