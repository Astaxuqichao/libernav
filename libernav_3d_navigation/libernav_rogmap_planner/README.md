# libernav_rogmap_planner

基于共享 `RogMap::ConstPtr` 的三维 `RogAStarPlanner`。它在全局稀疏占据网格上进行
6/26 邻域 A*，支持 diagonal、Manhattan、Euclidean heuristic、footprint 碰撞和
规划时间限制。

## 目标区域

点击地面点云时，目标可能落在占据体素上。规划器支持在请求目标周围搜索最近有效
体素：

```yaml
goal_region_xy_radius: 0.5
goal_region_z_radius: 0.75
```

静态 PCD 默认没有自由空间射线，因此 bringup 将 `unknown_as_obstacle` 设为 `false`；
占据点和 footprint 碰撞仍然有效。目标被调整时，返回消息会说明实际采用的有效点。

## 配置与插件

```text
plugin: libernav_rogmap_planner/RogAStarPlanner
```

配置示例见 `config/rog_astar.yaml`，实际运行参数由
`libernav_bringup/params/rogmap_params.yaml` 装配。规划器只生成 `Trajectory3D`，不
发布速度，也不拥有地图线程。
