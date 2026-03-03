# Runbook (referencia rapida)

## Requisitos

- ROS Melodic (Ubuntu 18.04 / Jetson TX2)
- Python 2.7 (incluido con Melodic)
- Dependencias adicionales para vehiculo real:
  - `ros-melodic-serial`
  - `ros-melodic-urg-node`
  - `ros-melodic-joy`
  - `ros-melodic-joy-teleop`

## Compilar

```bash
cd ~/tfg_racecar_ws
catkin_make
source devel/setup.bash
```

## Simulacion

```bash
roslaunch racecar_sim_control fixed_path_control.launch
```

Argumentos utiles:

```bash
roslaunch racecar_sim_control fixed_path_control.launch gui:=false enable_rviz:=false
```

## Vehiculo real

```bash
roslaunch racecar_sim_control real_vehicle_control.launch
```

Antes de lanzar, verificar hardware:

```bash
ls -l /dev/vesc /dev/hokuyo /dev/imu
```

Para el protocolo completo de despliegue, ver [LABORATORY_PLAYBOOK.md](LABORATORY_PLAYBOOK.md).

## Verificaciones rapidas

```bash
# Nodos activos
rosnode list | grep -E "vesc|cone|midpoints|centerline|pure_pursuit|mux"

# Topics del pipeline
rostopic list | grep -E "/perception/|/estimation/|/planning/|/control/"

# Frecuencias
rostopic hz /scan
rostopic hz /planning/pure_pursuit/delta
rostopic hz /control/ackermann_cmd_mux/input/navigation

# TF critica
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link laser

# Comando de control actual
rostopic echo -n 3 /control/ackermann_cmd
```

## Grabacion de datos

```bash
mkdir -p ~/bags/$(date +%Y%m%d_%H%M)
rosbag record -O ~/bags/$(date +%Y%m%d_%H%M)/run.bag \
  /scan /vesc/odom /tf /tf_static \
  /perception/lidar/cones \
  /estimation/track/centerline \
  /planning/track/centerline_path \
  /planning/pure_pursuit/delta \
  /planning/pure_pursuit/kappa \
  /planning/pure_pursuit/lookahead_distance \
  /control/ackermann_cmd_mux/input/navigation \
  /control/ackermann_cmd
```

## Documentacion

| Documento | Contenido |
|-----------|-----------|
| [ARCHITECTURE_OVERVIEW.md](ARCHITECTURE_OVERVIEW.md) | Pipeline, paquetes, frames TF |
| [MSG_API.md](MSG_API.md) | Referencia de mensajes custom |
| [LABORATORY_PLAYBOOK.md](LABORATORY_PLAYBOOK.md) | Protocolo de despliegue y seguridad |
| [launch_architecture.md](launch_architecture.md) | Convencion de launch files |
| [ROS_NAMING_STANDARD.md](ROS_NAMING_STANDARD.md) | Convencion de nombrado |
