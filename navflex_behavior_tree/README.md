# navflex_behavior_tree

行为树层负责任务编排，不拥有地图，也不实现规划或控制算法。它通过 Navflex action
调用执行层，并将恢复行为、路径更新和反馈统一纳入 lifecycle。

## 包关系

- `navflex_rogmap_bt_navigator`：独立 ROGMap 三维 BT navigator，调用 RogAStar 和
  ScanController。
- `navflex_bt_navigator`：兼容性启动/调用封装，不替代三维 navigator。
- `navflex_bt_nodes`：GetPath、ExePath、Recovery 等可复用 BT 节点。
- `navflex_autonomous_exploration_bt`：基于 frontier 的探索任务行为树。

BT 节点通过 action 与 `navflex_nav` 通信，通过 blackboard 传递目标、轨迹、反馈和
恢复计数；地图查询仍留在 planner/controller 插件中。
