# navflex_frontier_planner

ROGMap-based frontier planning plugins for Navflex 3D navigation. The package
implements `navflex_rogmap_core::GlobalPlanner`.

The planner does not subscribe to a point cloud and does not build a private
occupancy map. `navflex_rog_map` consumes `PointCloud2` and passes the shared
`RogMap::ConstPtr` to each plugin. Frontier state, visibility, clearance, and
A* validity therefore use the same map as the Navflex planner server.

## Plugins

- `navflex_frontier_planner/CandidateFrontierPlanner` returns ranked reachable
  frontier viewpoints in a `Trajectory3D` path.
- `navflex_frontier_planner/FrontierAStarPlanner` returns a collision-free
  ROGMap A* trajectory to the best reachable frontier.

## Configuration

The package is configured in `navflex_bringup/params/rogmap_params.yaml` under
`frontier_shared_config`. The point cloud topic is configured only through
`rog_map.point_cloud_topic`.

```bash
colcon build --symlink-install --packages-up-to navflex_frontier_planner
```
