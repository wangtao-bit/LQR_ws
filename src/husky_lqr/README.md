# husky_lqr

ROS2 Humble 下的差速机器人路径跟踪示例包，核心是：

- 基于离散 LQR（DARE）的路径跟踪控制器 `lqr_tracker_node`
- 覆盖路径发布器 `bow_coverage_path_publisher`
- 跟踪指标统计节点 `tracking_metrics_node`
- RViz 可视化配置（路径、TF、前瞻点）




## 1. 目录结构

```text
husky_lqr/
├── CMakeLists.txt
├── package.xml
├── include/husky_lqr/
│   ├── lqr_tracker_node.hpp
│   └── unified_lqr_solver.hpp
├── launch/
│   ├── bow_coverage_tracking.launch.py
│   └── lqr_tracking.launch.py
├── rviz/
│   └── lqr_tracking.rviz
└── src/
    ├── bow_coverage_path_publisher.cpp
    ├── lqr_tracker_node.cpp
    ├── tracking_metrics_node.cpp
    └── unified_lqr_solver.cpp
```


## 2. 功能概览

### 2.1 lqr_tracker_node

订阅路径和里程计，输出速度指令：

- 订阅：`/plan` (`nav_msgs/Path`)、`/odom` (`nav_msgs/Odometry`)
- 发布：`/cmd_vel` (`geometry_msgs/Twist`)
- 可视化发布：`/lookahead_point_marker` (`visualization_msgs/Marker`)

控制逻辑要点：

- 统一 LQR 同时修正 `v` 和 `omega`（`delta_v`、`delta_w`）
- 低速线性化保护（`min_lqr_speed`）
- 曲率限速（`kappa` 相关）
- DARE 失败回退（复用历史增益或降级控制）


### 2.2 bow_coverage_path_publisher

参数化生成覆盖往返路径并持续发布：

- 发布：`/plan` (`nav_msgs/Path`)
- 发布可视化：`/bow_coverage_marker` (`visualization_msgs/Marker`)

路径形态：

- 往返覆盖线 + 连接段（当前为直角连接）
- 支持通过 `lane_spacing`、`length`、`width` 等参数控制密度与范围


### 2.3 tracking_metrics_node

用于跟踪效果评估：

- 订阅：`/plan`、`/odom`
- 日志输出：横向误差 RMS、最大误差、航向最大误差、完成率


## 3. 依赖与环境

- ROS2 Humble
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `std_msgs`
- `visualization_msgs`
- `Eigen3`


## 4. 构建与测试

在工作空间根目录执行：

```bash
cd ~/lqr_ws
colcon build --packages-select husky_lqr
source install/setup.bash
```

运行测试：

```bash
colcon test --packages-select husky_lqr
colcon test-result --verbose
```


## 5. 启动方式

### 5.1 覆盖路径 + 跟踪 + RViz（推荐）

```bash
ros2 launch husky_lqr bow_coverage_tracking.launch.py
```

该启动会拉起：

- `bow_coverage_path_publisher`
- `lqr_tracker_node`
- `tracking_metrics_node`（可通过 `use_metrics:=false` 关闭）
- `rviz2`（加载 `rviz/lqr_tracking.rviz`）


### 5.2 简化调试启动

```bash
ros2 launch husky_lqr lqr_tracking.launch.py
```

同样包含路径发布 + 跟踪 + RViz，但默认参数更偏向快速本地调试。


## 6. 常用参数

### 6.1 覆盖路径参数（`bow_coverage_path_publisher`）

- `frame_id`：路径坐标系（默认 `map`）
- `lane_spacing`：相邻覆盖线间距
- `length`：覆盖长度
- `width`：覆盖宽度
- `turn_radius`：转向几何参数
- `resolution`：路径离散分辨率（越小越密）
- `publish_rate`：路径发布频率
- `origin_x` / `origin_y`：路径起点偏移
- `pass_count`：指定往返条数（`0` 表示按 width 自动推算）
- `closed_loop`：是否闭环连接


### 6.2 跟踪参数（`lqr_tracker_node`）

- `control_rate`：控制频率
- `lookahead_dist`：前瞻距离
- `goal_tolerance`：终点停止阈值
- `base_v_ref`：基础参考速度
- `v_min` / `v_max`：线速度边界
- `w_max`：角速度边界
- `accel_limit` / `decel_limit`：线速度变化约束
- `w_accel_limit`：角速度变化约束
- `q_ex` / `q_ey` / `q_etheta`：状态误差权重
- `r_v` / `r_w`：控制量权重
- `min_lqr_speed`：LQR 线性化最低速度
- `kappa_speed_eps`：曲率限速稳定项


## 7. RViz 可视化说明

默认配置文件：`rviz/lqr_tracking.rviz`

重点显示：

- `PlanPath`：当前执行路径 `/plan`
- `LookaheadPoint`：当前前瞻点 `/lookahead_point_marker`
- `TF`：坐标关系
- `Map`：地图层（如你的系统提供）


## 8. 常见问题排查

### 8.1 机器人不动

优先检查：

- 是否有 `/plan` 数据
- 是否有 `/odom` 数据
- `/cmd_vel` 是否被其他节点覆盖

可用命令：

```bash
ros2 topic echo /plan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel
```


### 8.2 急弯抖动或绕圈

尝试：

- 降低 `base_v_ref`
- 增大 `r_w`（惩罚角速度变化）
- 适当减小 `lookahead_dist`
- 增大路径 `resolution`（更密的参考点）


### 8.3 前瞻点看不到

- 检查 RViz 里 `LookaheadPoint` 显示是否启用
- 确认 `Fixed Frame` 与路径坐标系一致（通常 `map`）
- 检查话题是否有数据：

```bash
ros2 topic echo /lookahead_point_marker --once
```


## 9. 版本说明

本 README 对应当前代码状态（已移除 RViz 点选构建路径功能）。

如果后续恢复交互式路径绘制，请同步更新本文件中的节点、话题与启动说明。
