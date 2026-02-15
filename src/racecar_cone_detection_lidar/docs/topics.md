# Topics e interfaces

## Detector LiDAR (`cone_detector`)

- Subscrito:
  - `~input/scan_topic` (`sensor_msgs/LaserScan`, default `/scan`)

- Publicado:
  - `~output/cones_topic` (`racecar_cone_msgs/ConeArray`, default `/perception/lidar/cones`)
  - `~output/markers_topic` (`visualization_msgs/MarkerArray`, default `/perception/lidar/cones/markers`)
  - `~debug/serial_topic` (`std_msgs/String`, default `/perception/lidar/cones/debug`)

## Error evaluator (`cone_error_evaluator`)

- Subscrito:
  - `~input/raw_topic` (`racecar_cone_msgs/ConeArray`, default `/perception/lidar/cones`)
  - `~input/gt_topic` (`racecar_cone_msgs/ConeArray`, default `/ground_truth/track/cones`)
