# scan_planner

Single-shot SCAN local planner and controller implemented as a
`navflex_rogmap_core::Controller` plugin. It is adapted to the NavFlex ROS 2
plugin lifecycle from the main planning ideas in
[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner), without its FSM.

The plugin consumes a RogAStar `nav_msgs/Path` as its reference and a shared
`RogMap::ConstPtr` as its only obstacle map. On the first control cycle for a
new path it performs one planning pass:

1. prune the traversed prefix and resample reference control points;
2. repair colliding portions with an XY A* search whose Z follows the local
   reference segment, matching SCAN-Planner's cross-floor search constraint;
3. optimize smoothness, collision clearance, reference fitness, and local
   feasibility costs using ROG-Map ESDF gradients;
4. sample a cubic B-spline and apply forward/backward velocity and acceleration
   time parameterization.

Execution uses feed-forward plus position/yaw feedback, heading-first rotation,
acceleration limiting, cancellation, and speed limits. It continuously validates
the planned lookahead against ROG-Map, but deliberately does not replan. If the
trajectory becomes blocked, the current action fails and a new goal is required.
For quadrupeds, path Z participates in planning and collision checking but is not
published as `cmd_vel.linear.z`.

Supported ROG-Map footprint types are `sphere`, `cylinder`, `box`, and
`double_sphere`.

Plugin type:

```text
scan_planner/ScanController
```

See `config/scan_controller.yaml` for the `navflex_nav` ROG-Map mode parameters.
