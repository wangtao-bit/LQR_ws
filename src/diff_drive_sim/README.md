# diff_drive_sim

2驱差速运动学仿真包，提供 `cmd_vel -> odom` 接口并支持 RViz 机器人模型可视化。

## 功能

- 订阅：`/cmd_vel` (`geometry_msgs/Twist`)
- 发布：`/odom` (`nav_msgs/Odometry`)
- 发布：`/joint_states` (`sensor_msgs/JointState`)
- 广播 TF：`odom -> base_link`

## 运行

```bash
cd ~/LQR_ws
colcon build --packages-select diff_drive_sim
source install/setup.bash
ros2 launch diff_drive_sim sim_rviz.launch.py
```

发送速度命令示例：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.4}}" -r 20
```

## 常用参数

- `wheel_base`：轮距（默认 `0.5`）
- `wheel_radius`：轮半径（默认 `0.1`）
- `update_rate`：仿真更新频率 Hz（默认 `50.0`）
- `cmd_timeout`：速度指令超时置零（默认 `0.5` 秒）
