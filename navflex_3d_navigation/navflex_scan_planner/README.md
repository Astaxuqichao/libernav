# navflex_scan_planner

ROGMap 后端的局部 `ScanController`。它把全局规划器的 `Trajectory3D` 转换为可执行
的局部轨迹，并用共享局部 ROGMap/ESDF 进行碰撞、平滑和控制。

## 控制链

```text
RogAStar path
   -> prune/resample
   -> local XY repair (Z follows reference)
   -> ESDF collision + smooth/reference/feasibility optimization
   -> cubic B-spline
   -> velocity/acceleration time parameterization
   -> TwistStamped
```

控制器会在新路径上执行一次 SCAN 优化，并持续验证 lookahead；当前轨迹被实时障碍
阻塞时返回错误，让上层行为树重新规划。它支持 sphere、cylinder、box 和
double_sphere footprint。路径的 Z 用于跨层和楼梯碰撞检查；是否发布垂直速度由底盘
接口决定，当前默认平面底盘不输出 `cmd_vel.linear.z`。

## 接口与参数

```text
plugin: navflex_scan_planner/ScanController
trajectory_topic: navflex_scan_planner/optimized_path
```

控制器实现 `navflex_rogmap_core::Controller`，通过 `setTrajectory()` 接收路径，
通过带消息的 `computeVelocityCommands()` 输出 `TwistStamped`。参数包括采样距离、
优化迭代、ESDF 碰撞权重、最大速度/加速度、lookahead 和 footprint，示例见
`config/scan_controller.yaml`，bringup 运行配置见 `rogmap_params.yaml`。
