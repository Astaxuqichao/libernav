# libernav_frontier_planner

ROGMap frontier 插件，用于在共享三维地图中选择可到达的未知边界。它不订阅点云、
不维护私有 occupancy map，而是直接查询 `libernav_rog_map::RogMap::ConstPtr`；因此
frontier 可见性、双球体 clearance、ESDF 和 A* 碰撞判断与其他 ROGMap 插件完全一致。

## 插件

- `CandidateFrontierPlanner`：提取局部 frontier，按信息增益、距离、clearance 和
  目标权重排序并生成候选轨迹。
- `FrontierAStarPlanner`：选择最佳可达候选，并在 ROGMap 全局/局部查询上执行 A*。

两者都实现 `libernav_rogmap_core::GlobalPlanner`，由 `libernav_nav` 的 planner server
统一加载和取消。参数集中在 `rogmap_params.yaml` 的 `frontier_shared_config`，
地图输入只在 `rog_map.rog_map` 配置一次。

## 运行和调试

```bash
colcon build --packages-up-to libernav_frontier_planner --symlink-install
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=rogmap
```

调试重点是 `frontier_extraction`、`local_range`、`min_frontier_cells`、候选数量和
footprint clearance；不要在该包中重新配置点云 topic 或地图分辨率。
