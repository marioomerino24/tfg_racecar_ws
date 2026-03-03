# API de Mensajes Custom (racecar_cone_msgs)

Referencia completa de los tipos de mensaje definidos en `racecar_cone_msgs/msg/`.

Todos los mensajes usan el frame `odom` como referencia espacial salvo que se indique lo contrario.

---

## Cone

Deteccion individual de un cono en coordenadas del frame fijo.

```
std_msgs/Header    header
geometry_msgs/Point p              # posicion 3D (z=0 en 2D)
float32[4]         cov_xy          # covarianza 2x2 aplanada row-major [xx, xy, yx, yy]
uint8              side            # lateralidad (ver constantes)
float32            confidence      # confianza de la deteccion [0..1]
string             source          # origen: "lidar", "sim", etc.
int32              id              # identificador de tracking (opcional, -1 si no aplica)
```

**Constantes de lateralidad:**

| Constante | Valor | Significado |
|-----------|-------|-------------|
| `UNKNOWN` | 0     | Sin clasificar |
| `LEFT`    | 1     | Borde izquierdo de pista |
| `RIGHT`   | 2     | Borde derecho de pista |

**Productores:** `cone_detector_node` (LiDAR), spawners de simulacion (ground truth).

**Notas:**
- `cov_xy` se calcula a partir de la dispersion del cluster LiDAR. Si el detector no puede estimarla, se rellena con ceros.
- `confidence` refleja la calidad geometrica del ajuste (Hyper fit > centroide).
- `source` permite distinguir detecciones reales de ground truth en evaluacion.

---

## ConeArray

Coleccion de conos detectados en un instante.

```
std_msgs/Header          header
racecar_cone_msgs/Cone[] cones
```

**Topic canonico:** `/perception/lidar/cones`

**Productores:** `cone_detector_node`

**Consumidores:** `midpoints_node`, `track_viz`

**Frecuencia tipica:** ~10-40 Hz (depende de la frecuencia del scan LiDAR).

---

## MidpointArray

Puntos medios de pista ordenados por avance longitudinal, con ancho local y covarianza.

```
std_msgs/Header          header
geometry_msgs/Point[]    points        # midpoints ordenados por s creciente
float32[]                width         # ancho local de pista en cada punto [m]
float32[]                cov_xy_flat   # covarianza 2x2 por punto, aplanada [xx,xy,yx,yy, ...]
int32[]                  id            # ID de bin (opcional)
float32[]                confidence    # confianza por punto [0..1]
```

**Topic canonico:** `/estimation/track/midpoints`

**Productores:** `midpoints_node`

**Consumidores:** `centerline_fit_node`, `track_viz`

**Invariantes:**
- `len(points) == len(width) == len(id) == len(confidence)`
- `len(cov_xy_flat) == 4 * len(points)`
- `points` estan ordenados por distancia longitudinal al vehiculo (s creciente).

**Notas:**
- Los puntos se generan discretizando el espacio longitudinal en bins de `ds_bin` metros (por defecto 0.5 m).
- `width` es la distancia entre bordes izquierdo y derecho en cada bin.
- La covarianza se estima con mediana/MAD (resistente a outliers).

---

## Centerline

Spline cubica natural que representa la linea central de pista, remuestreada uniformemente.

```
std_msgs/Header          header
geometry_msgs/Point[]    samples            # N puntos de la trayectoria c(s)
float32[]                s                  # longitud de arco acumulada [m], s[0]=0
float32[]                kappa              # curvatura por muestra [1/m], signo segun convencion
bool                     is_arclength_uniform  # true si s es uniforme (paso constante ds)
float32                  ds                 # paso nominal [m] (0 si no uniforme)
geometry_msgs/Point[]    ctrl_pts           # puntos de control del ajuste (vacio si no aplica)
```

**Topic canonico:** `/estimation/track/centerline`

**Productores:** `centerline_fit_node`

**Consumidores:** `centerline_to_path_node`, `track_viz`

**Invariantes:**
- `len(samples) == len(s) == len(kappa)`
- `s[0] == 0`, `s` es monotona creciente.
- Si `is_arclength_uniform == true`, entonces `s[i+1] - s[i] == ds` para todo i.

**Algoritmo:**
- Spline cubica natural (M[0] = M[n-1] = 0) resuelta con el algoritmo de Thomas O(n).
- La curvatura `kappa` se calcula analiticamete a partir de las derivadas primera y segunda de la spline.
- El remuestreo uniforme se hace a paso `ds` (por defecto 0.25 m, configurable en `centerline.yaml`).

**Convencion de signo de curvatura:**
- Positivo: giro a la izquierda (sentido antihorario).
- Negativo: giro a la derecha (sentido horario).

---

## TrackMetrics

Metricas de calidad de la estimacion de pista para monitorizacion en tiempo real.

```
std_msgs/Header header
float32 coverage              # fraccion de bins con datos validos [0..1]
float32 std_width              # desviacion estandar del ancho de pista [m]
float32 max_kappa              # curvatura maxima absoluta [1/m]
float32 integral_kappa_sq      # integral de kappa^2 sobre s [1/m] (suavidad)
float32 latency_ms             # latencia media del ciclo de estimacion [ms]
uint32  cones_raw_count        # conos crudos recibidos en el ciclo
uint32  pairs_count            # pares L-R formados
uint32  midpoints_count        # midpoints publicados
string  notes                  # diagnostico libre
```

**Topic canonico:** `/estimation/track/metrics`

**Productores:** `midpoints_node`

**Interpretacion de campos:**
- `coverage` cercano a 1.0 indica buena visibilidad de pista.
- `std_width` elevado indica variabilidad en el ancho detectado (posible error de asociacion).
- `integral_kappa_sq` bajo indica trayectoria suave; alto indica curvas cerradas o ruido.
- `latency_ms` debe mantenerse por debajo de 50 ms para operacion en tiempo real.

---

## ConePair (DEPRECADO)

Par izquierdo-derecho de conos con midpoint y ancho.

```
std_msgs/Header          header
racecar_cone_msgs/Cone   left
racecar_cone_msgs/Cone   right
geometry_msgs/Point      midpoint
float32                  width       # distancia entre left y right [m]
```

**Estado:** Deprecado. Sustituido por la logica de binning longitudinal de `midpoints_node`.

---

## ConePairArray (DEPRECADO)

Coleccion de pares de conos.

```
std_msgs/Header              header
racecar_cone_msgs/ConePair[] pairs
```

**Estado:** Deprecado. Se mantiene para compatibilidad de compilacion pero no se usa en el pipeline vigente.

---

## Mensajes estandar ROS usados en el pipeline

Ademas de los mensajes custom, el pipeline usa estos tipos estandar en topics clave:

| Topic | Tipo | Descripcion |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Scan LiDAR crudo (Hokuyo) |
| `/vesc/odom` | `nav_msgs/Odometry` | Odometria del vehiculo (VESC o Gazebo) |
| `/planning/track/centerline_path` | `nav_msgs/Path` | Trayectoria de referencia para Pure Pursuit |
| `/planning/pure_pursuit/delta` | `std_msgs/Float64` | Angulo de direccion comandado [rad] |
| `/planning/pure_pursuit/kappa` | `std_msgs/Float64` | Curvatura local [1/m] |
| `/planning/pure_pursuit/lookahead_distance` | `std_msgs/Float64` | Distancia de lookahead adaptativa [m] |
| `/control/ackermann_cmd_mux/input/navigation` | `ackermann_msgs/AckermannDriveStamped` | Comando de control final |
| `/control/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Salida del mux hacia actuadores |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Transformaciones de frames |

---

## Diagrama de flujo de mensajes

```
 /scan (LaserScan)
   |
   v
 cone_detector_node
   |
   v
 /perception/lidar/cones (ConeArray)
   |
   v
 midpoints_node ---------> /estimation/track/metrics (TrackMetrics)
   |
   v
 /estimation/track/midpoints (MidpointArray)
   |
   v
 centerline_fit_node
   |
   v
 /estimation/track/centerline (Centerline)
   |
   v
 centerline_to_path_node
   |
   v
 /planning/track/centerline_path (nav_msgs/Path)
   |
   +--- pure_pursuit_target_selector ---> /planning/pure_pursuit/{delta,kappa,Ld}
   |                                           |
   |                                           v
   |                                  pure_pursuit_controller
   |                                           |
   |                                           v
   +------------------------------------> /control/ackermann_cmd_mux/input/navigation
                                               |
                                               v
                                          ackermann_mux
                                               |
                                               v
                                       /control/ackermann_cmd
```
