# Playbook de Laboratorio (Jetson TX2)

Guía operativa completa para desplegar y validar `tfg_racecar_ws` en coche real.

Objetivo: minimizar riesgo, detectar fallos temprano y generar evidencia técnica reproducible.

---

## 1) Preparación antes de salir al laboratorio

### 1.1 Material y seguridad

- E-stop físico verificado.
- Batería del coche cargada y batería de mando/joystick cargada.
- Teclado + ratón + HDMI para la TX2.
- Espacio de pruebas despejado y delimitado.
- Conos/escenario preparado.

### 1.2 Estado del software (en portátil antes de copiar a TX2)

- Workspace compilable en limpio.
- Configuración final congelada (commit o snapshot identificable).
- Launch objetivo definido:
  - Coche real: `roslaunch racecar_sim_control real_vehicle_control.launch`

### 1.3 Dependencias esperadas en TX2 (ROS Melodic)

- `ros-melodic-serial`
- `ros-melodic-urg-node`
- `ros-melodic-joy`
- `ros-melodic-joy-teleop`

---

## 2) Checklist de llegada al laboratorio (sin mover el coche)

## 2.1 Arranque base

```bash
cd ~/tfg_racecar_ws
catkin_make
source devel/setup.bash
```

Comprobar que no hay errores de compilación ni de entorno.

## 2.2 Verificación de puertos hardware

```bash
ls -l /dev/vesc /dev/hokuyo /dev/imu
```

Si alguno no existe:
- revisar cableado,
- revisar alimentación,
- revisar reglas `udev` (si aplica),
- reiniciar dispositivo/sensor.

## 2.3 Lanzamiento del pipeline real

```bash
roslaunch racecar_sim_control real_vehicle_control.launch
```

Esperar 10-20 s y verificar que no hay bucles de crash/restart en consola.

---

## 3) Validaciones técnicas mínimas (GO/NO-GO)

## 3.1 Nodos críticos activos

```bash
rosnode list | rg "vesc|imu|laser|cone|midpoints|centerline|pure_pursuit|ackermann"
```

Esperado: aparecen nodos de sensado, percepción, estimación, pure pursuit, control y mux.

## 3.2 Tópicos críticos presentes

```bash
rostopic list | rg "/scan|/vesc/odom|/perception/lidar/cones|/planning/track/centerline_path|/planning/pure_pursuit|/control/ackermann_cmd"
```

## 3.3 Frecuencias mínimas

```bash
rostopic hz /scan
rostopic hz /vesc/odom
rostopic hz /planning/pure_pursuit/delta
rostopic hz /control/ackermann_cmd_mux/input/navigation
```

Criterio recomendado:
- frecuencia estable (sin huecos largos),
- no caer a 0 Hz durante operación.

## 3.4 TF imprescindible (crítico)

```bash
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link laser
```

Debe existir transformación estable:
- `odom -> base_link` (dinámica),
- `base_link -> laser` (estática).

Si falla TF, **NO-GO** (no arrancar dinámica).

## 3.5 Salida de control segura

```bash
rostopic echo -n 5 /control/ackermann_cmd
```

Esperado:
- mensajes válidos,
- ángulos razonables,
- velocidad coherente con modo y fase de prueba.

---

## 4) Secuencia profesional de pruebas en pista

## 4.1 Fase A: estático (ruedas en suelo, coche inmóvil)

- Verificar percepción de conos en RViz.
- Verificar centerline/path publicado.
- Verificar que no hay comandos espurios grandes.

PASS si:
- percepción estable,
- TF estable,
- control sin picos.

## 4.2 Fase B: movimiento muy lento (zona despejada)

- Activar teleop con velocidad mínima.
- Confirmar respuesta de dirección y frenado.
- Confirmar prioridad de teleop/safety frente a navegación.

PASS si:
- respuesta suave,
- parada inmediata al soltar deadman / e-stop.

## 4.3 Fase C: navegación limitada

- Habilitar navegación en tramo corto.
- Velocidad capada.
- Observar:
  - continuidad de `/planning/pure_pursuit/*`,
  - regularidad de `/control/ackermann_cmd`.

PASS si:
- seguimiento estable sin oscilaciones severas,
- sin timeout recurrente de entradas.

## 4.4 Fase D: operación nominal

- Solo tras PASS en A, B y C.
- Incrementar velocidad gradualmente.
- Mantener operador en condición de e-stop inmediato.

---

## 5) Criterios GO/NO-GO recomendados

## GO (todos deben cumplirse)

- Sin crashes de nodos críticos durante al menos 5 minutos.
- TF estable en toda la cadena crítica.
- `/vesc/odom` y `/scan` con frecuencia estable.
- Control sin comandos erráticos.
- E-stop y deadman verificados.

## NO-GO (cualquiera de estos)

- Falta `odom -> base_link`.
- Caídas de nodos críticos repetidas.
- Timeouts constantes de percepción/control.
- Comandos de velocidad/dirección no coherentes.
- Cualquier duda de seguridad operativa.

---

## 6) Registro de datos (obligatorio para trazabilidad)

Crear una carpeta por sesión:

```bash
mkdir -p ~/bags/$(date +%Y%m%d_%H%M)_lab_session
cd ~/bags/$(date +%Y%m%d_%H%M)_lab_session
```

Grabación mínima:

```bash
rosbag record -O run.bag \
  /scan \
  /vesc/odom \
  /tf /tf_static \
  /perception/lidar/cones \
  /estimation/track/centerline \
  /planning/track/centerline_path \
  /planning/pure_pursuit/delta \
  /planning/pure_pursuit/kappa \
  /planning/pure_pursuit/lookahead_distance \
  /control/ackermann_cmd_mux/input/navigation \
  /control/ackermann_cmd
```

Guardar junto al bag:
- launch usado,
- parámetros usados,
- observaciones de pista,
- incidencias.

---

## 7) Protocolo de incidencia y recuperación

## 7.1 Si se pierde percepción

- Reducir inmediatamente a velocidad segura.
- Revisar `/scan`, frame `laser`, y `cone_detector`.
- Si no recupera en segundos: parar prueba.

## 7.2 Si se pierde TF

- Parar prueba.
- Comprobar `vesc_to_odom` y static transforms.
- No reanudar sin `tf_echo` correcto.

## 7.3 Si el control manda valores erráticos

- E-stop.
- Verificar tópicos de entrada de control.
- Revisar timeouts y latencias.
- Reanudar solo tras validar en estático.

---

## 8) Cierre de sesión de laboratorio

- Parada segura del coche.
- Guardar bags y logs.
- Anotar resultados PASS/FAIL por fase (A/B/C/D).
- Identificar acciones correctivas concretas para la siguiente sesión.

Plantilla mínima de cierre:
- Fecha/hora:
- Launch:
- Config:
- Resultado A:
- Resultado B:
- Resultado C:
- Resultado D:
- Incidencias:
- Decisión final (GO/NO-GO):

---

## 9) Notas específicas para este workspace

- El pipeline real depende de:
  - `racecar` (bringup físico),
  - `vesc/*`,
  - `ackermann_cmd_mux`,
  - `razor_imu_9dof`.
- `pure_pursuit_control` incluye watchdog por timeout de entradas; si caducan señales, publica `stop`.
- `vesc_to_odom/publish_tf` debe permanecer activo para la cadena TF crítica del control.

