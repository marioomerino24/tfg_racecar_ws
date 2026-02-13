std_msgs/Header header

# Cobertura de emparejado (0..1)
float32 coverage

# Estadística de anchos
float32 std_width                      # desviación estándar de width (m)

# Curvatura y suavidad
float32 max_kappa                      # max |kappa| (1/m)
float32 integral_kappa_sq              # ∫ kappa^2 ds (1/m)

# Rendimiento
float32 latency_ms                     # latencia media del ciclo (ms)

# Contadores
uint32 cones_raw_count
uint32 pairs_count
uint32 midpoints_count

# Notas/diagnóstico
string notes
