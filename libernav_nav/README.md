# libernav_nav

`libernav_nav` 是 LiberNav 的统一导航执行层。它把 costmap 和 ROGMap 后端放在同一
个 package 中，提供相同的 lifecycle、action、取消、重试、超时、错误消息和插件
加载流程。

## 后端装配

```text
CostmapNavNode -> nav2_core plugins -> Nav2 costmap
RogMapNavNode  -> libernav_rogmap_core plugins -> RogMap::ConstPtr
```

`navigation_type` 由 bringup 选择 `costmap`、`rogmap` 或 `both`。双后端使用独立
命名空间，避免 action、地图和控制输出互相覆盖：

| 后端 | 规划 | 控制 | 恢复 |
| --- | --- | --- | --- |
| costmap | `/costmap/compute_path_to_pose` | `/costmap/follow_path` | `/costmap/behavior_action` |
| rogmap | `/rogmap/compute_path_to_pose` | `/rogmap/follow_path` | `/rogmap/behavior_action` |

两套控制输出不能同时直接连接底盘，应通过 mux 或明确选择一个后端。

## 共用执行框架

`include/libernav_base` 保存 planner/controller/recovery 的执行状态机和 plugin adapter，
`src/costmap_nav`、`src/rogmap_nav` 只处理后端资源装配。规划和控制插件不负责
action 生命周期，也不直接创建地图线程。

ROGMap 后端的地图由 `libernav_rog_map::RogMapROS` 创建一次，并以只读
`RogMap::ConstPtr` 传给规划器和控制器。这样全局规划和局部控制共享同一份占据、
ESDF、footprint 和地图版本，不需要复制点云或在插件间转换地图格式。

## 主要接口

- planner：`makePlan(start, goal, trajectory, message)`，返回 `uint32_t` 错误码。
- controller：`setTrajectory()` 和带 `message` 的 `computeVelocityCommands()`。
- recovery：`runBehavior(name, message)`，在同一 lifecycle 下运行。
- 所有插件必须实现 `configure -> activate -> deactivate -> cleanup`。

## 操作

先启动 costmap 后端：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=costmap
```

启动 ROGMap 后端：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=rogmap
```

详细实现位于 `include/libernav_base`、`include/libernav_nav/{costmap_nav,rogmap_nav}`
以及对应的 `src` 目录。
