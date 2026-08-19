# libernav_bringup

统一部署包，集中管理 launch、参数、内置地图、RViz 配置和仿真入口。算法包不在
这里创建地图或规划线程；bringup 只负责选择后端、装配插件和安排 lifecycle 顺序。

## 启动关系

```text
libernav_bringup_launch.py
  ├── libernav_nav::CostmapNavNode   (navigation_type=costmap)
  ├── libernav_nav::RogMapNavNode    (navigation_type=rogmap)
  ├── libernav_rogmap_bt_navigator   (use_bt_navigator=true)
  └── lifecycle_manager_libernav
```

默认启动 costmap 并启用 costmap 行为树导航器。选择 `navigation_type:=rogmap` 时仅启动 ROGMap
及其行为树；选择 `navigation_type:=both` 时两套后端和行为树都会启动。ROGMap 使用内置 PCD `maps/multilevel_ramp_stairs_0p1m.pcd`，其
全局地图默认静态，局部 ROGMap 仍接收运行时传感器点云。Costmap 和 ROGMap 可以用
`navigation_type:=both` 同时启动，但两个控制输出必须经过速度 mux 才能连接同一底盘。

## 文件结构

```text
launch/
  libernav_bringup_launch.py          后端选择、组件和 lifecycle
  pcd_terrain_simulator_launch.py    只启动 PCD 仿真、odom、TF 和 RViz
  sim_local_launch.py                轻量底盘/传感器仿真
params/
  rogmap_params.yaml                 ROGMap、RogAStar、ScanController 装配
  nav2_params*.yaml                  costmap 后端参数
maps/                                 PCD、语义和二维投影
rviz/                                 PCD 3D 和 costmap RViz 配置
tools/                                地图生成和渲染脚本
```

## 最短操作

默认启动 costmap 和对应行为树，验证二维导航链路：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py
```

如需只启动 costmap 后端但不启用行为树，可显式关闭行为树：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py use_bt_navigator:=false
```

二维本地仿真与导航独立运行。先启动地图、全向底盘、仿真激光、TF 和 RViz：

```bash
ros2 launch libernav_bringup sim_local_launch.py
```

再在另一终端启动 costmap 导航：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=costmap
```

启动 ROGMap：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=rogmap
```

只启动 PCD 仿真（不启动导航）：

```bash
ros2 launch libernav_bringup pcd_terrain_simulator_launch.py
```

启动 ROGMap 和对应三维行为树：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=rogmap use_bt_navigator:=true
```

## ROGMap 参数组织

`rogmap_params.yaml` 同时描述地图服务器和插件装配，避免 planner/controller 各自
维护地图副本：

```text
rog_map.rog_map                 global/local resolution、PCD、传感器、ESDF
libernav_planner_server.RogAStar 目标区域、未知空间、三维 footprint、搜索预算
libernav_controller_server.ScanController 轨迹优化、速度/加速度、footprint
```

常用静态地图设置：

```yaml
load_pcd: true
enable_global_map_updates: false
enable_raycasting: true
```

## 三维目标

PCD RViz 配置默认使用 `Move Camera`，不会自动进入 `Publish Point`。工具栏仍保留
`Publish Point`，手动选择后点击点云会发布 `/clicked_point`。构建并 source
`libernav_rviz_plugins` 后，RViz 还可以加载 `3D Goal` 面板，直接输入 Frame、X、Y、
Z 和 Yaw（弧度），发布 `/goal_pose`。

对当前 PCD 地形，目标 Z 应填写机器人中心高度，而不是地面点高度；仿真默认离地
高度为 `0.5m`。ROG A* 的 `goal_region_xy_radius` 和 `goal_region_z_radius` 可在
点击地面或输入近地面坐标时寻找最近有效目标体素。

## 地图和仿真资源

- `tools/generate_multilevel_terrain_pcd.py`：生成 0.1m 体素地形、墙、斜坡和楼梯。
- `tools/render_multilevel_terrain_topdown.py`：从 PCD 生成二维最高点投影。
- `libernav_3d_pcd_simulator`：读取 PCD，保持机器人与地面距离并发布 odom/TF。

仿真包只发布运动学状态，不伪造传感器；真实点云可直接接入 ROGMap 的
`point_cloud_topic`。
