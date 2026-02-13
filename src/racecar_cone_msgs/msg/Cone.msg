# Detección de un cono (frame fijo: odom/map)
std_msgs/Header header
geometry_msgs/Point p           # posición (z=0 si 2D)
# Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]
float32[4] cov_xy

# Lateralidad
uint8 UNKNOWN=0
uint8 LEFT=1
uint8 RIGHT=2
uint8 side

float32 confidence              # [0..1]
string source                   # "lidar", "sim", etc.
int32 id                        # opcional (tracking)
