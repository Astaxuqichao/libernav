# navflex_3d_navigation

三维导航功能集合。该目录按“地图拥有者 -> 插件契约 -> 算法插件”分层：

```text
navflex_rog_map       地图输入、占据、滑窗、ESDF、footprint
        |
navflex_rogmap_core   planner/controller/recovery 抽象
        |
navflex_rogmap_planner / navflex_scan_planner / navflex_frontier_planner
```

所有算法共享 `navflex_rog_map::RogMap::ConstPtr`，不重复定义地图和占据枚举。全局
规划使用持久化稀疏占据索引，局部控制使用高分辨率 ROGMap 和 ESDF；同一份 footprint
查询同时服务 A*、SCAN 和恢复行为。

三维包不负责启动顺序。启动和参数由 `navflex_bringup` 管理，算法包只提供 pluginlib
插件和可复用库。

## 子包

- `navflex_rog_map`：ROS 2 lifecycle 地图服务器和 C++ 查询 API。
- `navflex_rogmap_core`：三维插件接口和 `Trajectory3D` 数据类型。
- `navflex_rogmap_planner`：ROGMap 全局 A*。
- `navflex_scan_planner`：基于 ESDF 的局部 SCAN/B-spline 控制。
- `navflex_frontier_planner`：frontier 候选和探索规划。
- `navflex_rviz_plugins`：手动输入三维目标的 RViz Panel。
