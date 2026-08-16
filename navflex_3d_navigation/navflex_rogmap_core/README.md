# navflex_rogmap_core

ROGMap 后端的插件契约层。它刻意不拥有地图数据、不实现 ROS 传感器回调，也不复制
占据网格；所有插件通过 `navflex_rog_map::RogMap::ConstPtr` 读取同一张地图。

## 数据流

```text
PCD / PointCloud2 / TF
        -> navflex_rog_map::RogMapROS
        -> RogMap (global occupancy + local ROGMap + ESDF)
        -> GlobalPlanner / Controller / Recovery
```

## 接口

- `GlobalPlanner::makePlan(start, goal, Trajectory3D &, message)`：返回 `uint32_t`。
- `Controller::setTrajectory()` 与带消息输出的 `computeVelocityCommands()`：输出
  `TwistStamped`，可包含三维速度。
- `Recovery::runBehavior(name, message)`：执行恢复行为并返回错误码。

所有接口都遵循 `configure`、`activate`、`deactivate`、`cleanup` 生命周期，并使用
Nav2 风格的字符串消息和取消语义。pluginlib 的 base class 分别是：

```text
navflex_rogmap_core::GlobalPlanner
navflex_rogmap_core::Controller
navflex_rogmap_core::Recovery
```

地图状态、ESDF 和 sphere/cylinder/box/double_sphere footprint 语义只在
`navflex_rog_map` 定义；插件只组合这些能力。
