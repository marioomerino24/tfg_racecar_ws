std_msgs/Header header                 # frame_id típico: "odom"

# Muestras de la trayectoria c(s) remuestreadas
geometry_msgs/Point[] samples          # tamaño N

# Longitud de arco acumulada para cada sample (m)
float32[] s                            # tamaño N, s[0]=0

# Curvatura por muestra (1/m). Signo según convención (yaw izquierda +)
float32[] kappa                        # tamaño N

# Metadatos de muestreo
bool is_arclength_uniform              # true si s es uniforme
float32 ds                             # paso nominal en s (m), 0 si no aplica

# (Opcional) puntos de control del ajuste (vacío si no aplica)
geometry_msgs/Point[] ctrl_pts
