# libernav_simulation

仿真包提供最小运动学和传感器替身，不改变 LiberNav 的地图/规划接口：

- `libernav_3d_pcd_simulator`：读取原始 PCD，机器人保持可配置离地高度，接收速度并
  发布 odom、TF 和点云地图，用于跨层/楼梯运动学测试。
- `omni_fake_node`：二维全向底盘和 `/cmd_vel`、`/odom`、TF、`/clock`。
- `simulation_lidar`：从二维占据图生成 `/scan` 和 `/scan_cloud`。

二维本地仿真由 `libernav_bringup/sim_local_launch.py` 装配：它启动全向底盘、二维
map server、仿真激光、`map -> odom -> base_link` TF 和 RViz。随后单独启动
`navigation_type:=costmap` 的 LiberNav 后端，即可用同一 `/map`、`/scan`、`/odom`
验证 costmap planner/controller。

仿真只提供输入和运动学状态，真实 planner/controller 仍通过 `libernav_nav` 和共享
地图运行，因此替换为真实传感器不需要改算法包。
