# navflex_rviz_plugins

RViz integration for manual Navflex 3D goals. `Goal3DPanel` is a standard
`rviz_common::Panel` plugin with editable Frame, Topic, X, Y, Z and Yaw fields.
Yaw is in radians and the panel publishes `geometry_msgs/PoseStamped` (default
topic `/goal_pose`).

The panel only publishes a goal; it does not depend on ROGMap or call an action
server. `navflex_rogmap_bt_navigator` consumes the topic and owns TF conversion,
behavior-tree execution, cancellation and feedback. This keeps RViz optional
and allows the same input panel to be used with another goal consumer.

The PCD RViz configuration also retains the standard `Publish Point` tool for
surface picking. The default tool is `Move Camera`, so starting RViz does not
publish a goal or enter an interactive-marker mode.

Build the package from the workspace root and source the resulting overlay
before starting RViz:

```bash
colcon build --packages-select navflex_rviz_plugins --symlink-install
source install/setup.bash
```
