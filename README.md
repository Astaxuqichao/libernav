# LiberNav

LiberNav 是面向 ROS 2 Humble 的模块化导航框架。它保留 Nav2 的 lifecycle、action
和 pluginlib 使用方式，同时把地图、规划、控制和行为树组织成可替换的后端。当前
后端包括二维 Nav2 costmap 和三维 ROG-Map。

LiberNav 的核心是 `libernav_nav`，不是某一种地图。`libernav_nav` 将 Nav2 风格的
lifecycle、action server、执行线程、取消、重试、错误码和插件装配收敛为一套通用
执行框架；costmap 与 ROGMap 只是该框架中的后端实现。

## 核心架构

```text
                    libernav_bringup / BT / application
                                      |
                                libernav_nav
        lifecycle + action + execution + retry + cancel + plugin adapters
                     /                                      \
          CostmapNavNode                                  RogMapNavNode
  shared global/local costmap                       shared RogMap::ConstPtr
        nav2_core plugins                         libernav_rogmap_core plugins
```

`libernav_nav` 是统一执行层。它负责生命周期、action server、线程、取消、重试、
错误消息和插件适配；后端节点只负责装配不同地图和插件：

```text
CostmapNavNode -> nav2_core plugins -> Nav2 costmap
RogMapNavNode  -> libernav_rogmap_core plugins -> shared RogMap
```

两条后端使用相同的执行状态机和 action 语义，但保持自己的地图查询和插件契约。
ROGMap 模式不启动 `nav2_bt_navigator`，而由 `libernav_rogmap_bt_navigator` 独立
加载三维行为树，直接调用 `RogAStar` 和 `ScanController`。

### 通用抽象与地图共享

- **执行抽象**：规划、控制、恢复都经过 `libernav_base` 的共用 action/execution
  状态机。后端差异被 adapter 隔离，不会蔓延到行为树和部署层。
- **地图共享**：每个后端由地图节点拥有地图和更新线程，并将同一地图实例交给该后端
  的 planner、controller、recovery。插件只查询地图，不创建私有副本。
- **二维后端**：`CostmapNavNode` 共享 Nav2 global/local costmap，并直接使用
  `nav2_core` 的 planner/controller/behavior 接口。
- **三维后端**：`RogMapNavNode` 共享 `RogMap::ConstPtr`；`libernav_rog_map` 负责
  PCD、点云、稀疏全局占据、局部滑窗、ESDF 和 footprint 查询，
  `libernav_rogmap_core` 只定义三维插件契约。
- **一致的行为语义**：后端都遵循 configure/activate/deactivate/cleanup 和一致的
  action 取消、重试、消息输出。应用层通过稳定命名空间选择后端，而不是绑定某种地图。

ROGMap 还提供 `sphere`、`cylinder`、`box` 和 `double_sphere` footprint；全局 PCD
可以静态保留，局部图继续融合传感器点云。这些是三维后端能力，不是 LiberNav 执行框架
的前提。

## 源码布局

```text
libernav/
├── libernav/                         metapackage
├── libernav_bringup/                 launch、参数、PCD、RViz
├── libernav_nav/                     共用执行层与 costmap/rogmap 后端
├── libernav_3d_navigation/
│   ├── libernav_rog_map/             ROGMap ROS 2 lifecycle 地图服务器
│   ├── libernav_rogmap_core/         三维插件接口
│   ├── libernav_rogmap_planner/      RogAStar 全局规划器
│   ├── libernav_scan_planner/        SCAN/BSpline 局部控制器
│   ├── libernav_frontier_planner/    frontier 规划器
│   └── libernav_rviz_plugins/        RViz 三维目标面板
├── libernav_behavior_tree/
│   ├── libernav_rogmap_bt_navigator/ 独立三维 BT navigator
│   └── libernav_bt_nodes/             GetPath、ExePath、Recovery 节点
├── libernav_simulation/              PCD 地形、底盘和传感器仿真
├── libernav_common/                  utility 和日志
└── libernav_costmap_navigation/      costmap 扩展插件
```

各层依赖方向为：`bringup/app -> BT/libernav_nav -> backend map/core -> third-party`。
地图层不依赖 bringup、仿真或具体 planner，插件也不拥有地图生命周期。

## 获取源码

在工作区外层获取 Nav2、时空体素层和 LiberNav：

```bash
mkdir -p ~/ros2/nav_ws
cd ~/ros2/nav_ws
git clone --branch humble --single-branch https://github.com/Astaxuqichao/navigation2.git navigation2
git clone --branch humble --single-branch https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git spatio_temporal_voxel_layer
git clone https://github.com/Astaxuqichao/libernav.git libernav
```

LiberNav 使用 `Astaxuqichao/navigation2` 中与 FollowPath/action 相关的扩展接口，
不要用系统中不匹配的 Nav2 替代。

## 编译

所有 build/install/log 都位于 `nav_ws` 外层，不要在 `libernav/` 源码目录编译：

```bash
cd ~/ros2/nav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths navigation2 spatio_temporal_voxel_layer libernav --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动

先启动 costmap 后端。它使用 `CostmapNavNode`、Nav2 costmap 和 `nav2_core` 插件，
是验证 LiberNav 通用执行架构的基础方式：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=costmap
```

二维仿真与导航栈分开启动。先启动全向底盘、地图、二维激光、TF 和 RViz：

```bash
ros2 launch libernav_bringup sim_local_launch.py
```

然后在另一个已 source 工作区的终端启动 costmap 后端：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=costmap
```

显式启动 ROGMap；默认 PCD 和全局静态地图参数由 bringup 提供：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py navigation_type:=rogmap
```

只启动 PCD 地形、里程计和 TF 仿真：

```bash
ros2 launch libernav_bringup pcd_terrain_simulator_launch.py
```

需要三维行为树时，在仿真之外单独启动：

```bash
ros2 launch libernav_bringup libernav_bringup_launch.py use_bt_navigator:=true
```

## 包文档

- [libernav_nav](libernav_nav/README.md)：统一执行层、后端节点和 action 命名空间
- [libernav_rog_map](libernav_3d_navigation/libernav_rog_map/README.md)：地图数据流、
  全局/局部语义和生命周期
- [libernav_rogmap_core](libernav_3d_navigation/libernav_rogmap_core/README.md)：插件契约
- [libernav_rogmap_planner](libernav_3d_navigation/libernav_rogmap_planner/README.md)：
  三维 A*、目标区域和 footprint
- [libernav_scan_planner](libernav_3d_navigation/libernav_scan_planner/README.md)：局部
  SCAN、B-spline、ESDF 控制链
- [libernav_rogmap_bt_navigator](libernav_behavior_tree/libernav_rogmap_bt_navigator/README.md)：
  独立三维行为树
- [libernav_bringup](libernav_bringup/README.md)：启动入口和参数组织
- [3D navigation](libernav_3d_navigation/README.md)：三维包关系
