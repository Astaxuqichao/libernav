# navflex_rogmap_bt_navigator

独立的 ROGMap 三维行为树执行器。它不启动 `nav2_bt_navigator`，而是管理自己的
`NavigateToPose` action server，并在行为树中调用 Navflex 的 planner/controller
action：

```text
NavigateToPose / goal_pose / clicked_point
                  |
        navflex_rogmap_bt_navigator
                  |
       navigate_to_pose_rogmap.xml
          /                   \
 /rogmap/compute_path_to_pose  /rogmap/follow_path
       RogAStar                 ScanController
```

## 目标输入

- `nav2_msgs/action/NavigateToPose`：使用完整 `PoseStamped`，包括 Z。
- `/goal_pose`：RViz Panel 或其他节点发布的完整三维目标。
- `/clicked_point`：RViz `Publish Point` 发布的点，节点负责 TF 转换并使用
  `clicked_point_goal_yaw` 生成姿态。

目标处理、TF 校验、行为树加载、反馈、取消和 recovery 都在本节点内完成，不需要
goal bridge。默认行为树位于 `behavior_trees/navigate_to_pose_rogmap.xml`，参数位于
`params/rogmap_bt_navigator.yaml`。

## 启动

独立启动 ROGMap 后，再启用该 navigator：

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py use_bt_navigator:=true
```

节点必须与 `/rogmap/navflex_nav` 使用同一命名空间和 action 名称。它只编排任务，
不直接读取地图；地图查询由 planner/controller 通过共享 `RogMap::ConstPtr` 完成。
