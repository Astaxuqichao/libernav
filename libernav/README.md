# libernav

`libernav` 是 LiberNav 导航栈的 ROS 2 metapackage。该包不提供节点或库，
仅通过运行依赖聚合仓库内的 LiberNav 功能包，便于统一安装和构建。

在工作区根目录中构建整套 LiberNav：

```bash
colcon build --packages-up-to libernav
```
