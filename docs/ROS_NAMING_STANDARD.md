# ROS Naming Standard

## Scope

This standard defines naming conventions for topics, nodes, parameters, and launch files.

## General rules

- Use English technical terms.
- Use `snake_case`.
- Use domain-based namespaces.

## Topic domains

- `/perception/...`
- `/estimation/...`
- `/planning/...`
- `/control/...`
- `/visualization/...` (optional)

## Canonical control topics

- `/control/ackermann_cmd_mux/input/navigation`
- `/control/ackermann_cmd_mux/input/teleop`
- `/control/ackermann_cmd_mux/input/safety`
- `/control/ackermann_cmd`

## Canonical pure pursuit topics

- `/planning/pure_pursuit/delta`
- `/planning/pure_pursuit/kappa`
- `/planning/pure_pursuit/lookahead_distance`

## Parameter style

Prefer hierarchical keys:

- `input/scan_topic`
- `output/cones_topic`
- `debug/topic`
- `control/enable_feature`

## Transition policy

Legacy topic names are removed from the final version.
All nodes and launches must use canonical topic names only.
