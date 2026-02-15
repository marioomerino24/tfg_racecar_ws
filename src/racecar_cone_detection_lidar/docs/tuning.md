# Tuning rapido del detector

Parametros en `config/detector.yaml`:

- `r_min`, `r_max`: rango util de medidas LiDAR.
- `eps_cluster`: distancia maxima entre puntos consecutivos para el cluster.
- `min_points`, `max_points`: tamano permitido de cluster.
- `cone_diam_min`, `cone_diam_max`: filtro geometrico de ancho.
- `pub_markers`: activa/desactiva marcadores RViz.

Parametros avanzados en el nodo:

- `min_circle_points`, `svd_eps`
- `hyper_r_max`, `hyper_min_points`, `hyper_min_angle_deg`

Recomendacion:

1. Ajustar primero filtros geometricos basicos (`eps_cluster`, `cone_diam_*`).
2. Ajustar despues el gating de Hyper para reducir falsos positivos lejanos.
3. Validar con `tools_error_eval.launch` en escenarios repetibles.
