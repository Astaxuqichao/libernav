# navflex_3d_pcd_simulator

Terrain-constrained 3D kinematic simulator for Navflex. It deliberately has no
sensor model: the PCD is a static reference surface, while navigation still
uses the real `navflex_rog_map` input path. The robot receives planar body-frame
velocity commands, follows the nearest continuous PCD support surface, and
keeps a constant configurable height above that surface.

The simulator publishes the original PCD once on `/pcd_map`, odometry on
`/odom`, a static `map -> odom` transform, and a dynamic `odom -> base_link`
transform. It also publishes `/clock` by default.

```bash
ros2 launch navflex_3d_pcd_simulator pcd_terrain_simulator.launch.py \
  pcd_file:=/absolute/path/to/terrain.pcd \
  ground_clearance:=0.5
```

The default terrain map is installed by `navflex_bringup` at
`maps/multilevel_ramp_stairs_0p1m.pcd`: a 10 m by 10 m, 0.1 m sampled test
map with 3 m outer walls and one connected terrain corridor: a 1 m high ramp,
a 1 m center platform, and a 1 m descending stepped section. The corridor has
2 m high, 0.2 m thick side walls and is away from the `(0, 0)` start position.
Regenerate it after changing the geometry with:

```bash
python3 ../navflex_bringup/tools/generate_multilevel_terrain_pcd.py
```

The node is intentionally independent of ROGMap. To test the complete stack,
start this package first and start `navflex_bringup` separately; both processes
share `/odom`, TF and the `map` frame. This separation lets the same simulator
drive costmap or ROGMap backends.

Inputs and outputs:

| Interface | Type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Uses `linear.x`, `linear.y`, and `angular.z` |
| `/pcd_map` | `sensor_msgs/msg/PointCloud2` | Original PCD map for RViz |
| `/odom` | `nav_msgs/msg/Odometry` | Simulated 3D position and measured vertical velocity |
| `map -> odom -> base_link` | TF | Map and robot coordinate relationships |

Key parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `ground_clearance` | `0.5` | Fixed robot height above the selected PCD support point |
| `ground_search_radius` | `0.35` | XY radius used to find PCD ground support |
| `max_ground_step` | `0.35` | Maximum allowed ground-height change per update; prevents floor jumps |
| `ground_grid_resolution` | `0.2` | PCD spatial-index cell size |
| `start_x`, `start_y`, `start_yaw` | `0.0` | Initial planar pose |
| `publish_clock` | `true` | Publish simulation time on `/clock` |

For stairs, set `max_ground_step` no lower than the expected rise and ensure
the PCD contains tread points at intervals smaller than `ground_search_radius`.
For multi-floor maps, keep `max_ground_step` below the floor-to-floor height
so the robot cannot accidentally switch floors at the same XY position.
