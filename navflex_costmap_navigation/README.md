# Costmap navigation

Plugins and behaviors that operate on the Nav2 two-dimensional costmap backend.

- Frontier planning moved to `navflex_3d_navigation/navflex_frontier_planner`
  because it now uses the shared ROGMap backend.
- `navflex_exclusion_zone`: runtime costmap exclusion zones.
- `navflex_cmdbehavior`: command-driven Nav2 behavior plugin.
