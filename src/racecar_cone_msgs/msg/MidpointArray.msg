std_msgs/Header header

# Puntos medios ordenados por avance
geometry_msgs/Point[] points

# Ancho local asociado a cada punto (m)
float32[] width

# Covarianza 2x2 de cada midpoint, aplanada (row-major) en bloques de 4:
# [xx, xy, yx, yy,  xx, xy, yx, yy, ...]  => longitud = 4 * N
float32[] cov_xy_flat

# IDs y confianza opcionales
int32[] id
float32[] confidence
