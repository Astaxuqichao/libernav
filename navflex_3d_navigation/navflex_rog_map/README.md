# navflex_rog_map

`navflex_rog_map` 是 Navflex 唯一的 ROGMap 地图拥有者，将原始 ROG-Map 能力包装为
ROS 2 lifecycle 节点和 C++ 查询 API。规划器、控制器和恢复器不各自加载地图，只共享
该节点提供的 `RogMap::ConstPtr`。

## 地图模型

```text
global grid:  持久化、稀疏、可由 PCD 初始化，用于全局规划
local ROGMap: 高分辨率滑动窗口，接收传感器射线和占据更新
ESDF:         局部窗口内高精度距离/梯度，窗口外回退全局距离
```

默认 bringup 使用内置 PCD 作为静态全局地图，`enable_global_map_updates=false`；
实时点云仍更新局部 ROGMap。需要在线维护全局图时将该参数设为 `true`。另外，
`enable_raycasting` 控制局部射线清除和命中/未命中累积。

## 输入与输出

- 输入：`sensor_msgs/PointCloud2`、传感器到 `global_frame` 的 TF、可选 PCD。
- 输出：`global_occupied`、`local_occupied` 两个 transient-local 点云话题。
- 查询：全局/局部占据、膨胀占据、frontier、碰撞、raycast、ESDF 距离和梯度。
- footprint：`sphere`、`cylinder`、`box`、`double_sphere`，规划与控制共用。

## 生命周期与参数

`RogMapROS` 可独立运行，也可作为 `RogMapNavNode` 的 child lifecycle node。配置顺序
与 Nav2 costmap 一致：创建参数 -> configure 地图 -> activate 输入和发布 -> cleanup。
主要参数集中在 `navflex_bringup/params/rogmap_params.yaml`：

```text
global_* / local_*             地图范围与分辨率
load_pcd / pcd_file            静态初始地图
enable_global_map_updates      全局图是否接受传感器改写
enable_raycasting              局部射线更新开关
enable_esdf                    ESDF 开关
```

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py navigation_type:=rogmap
```

地图接口实现集中在 `include/navflex_rog_map/rog_map.hpp` 和 `src/rog_map.cpp`，ROS
生命周期包装在 `rog_map_ros.*`。
