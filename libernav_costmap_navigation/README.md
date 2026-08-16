# Costmap navigation

Plugins and behaviors that operate on the Nav2 two-dimensional costmap backend.

- Frontier planning moved to `libernav_3d_navigation/libernav_frontier_planner`
  because it now uses the shared ROGMap backend.
- `libernav_exclusion_zone`: runtime costmap exclusion zones.
- `libernav_cmdbehavior`: command-driven Nav2 behavior plugin.
