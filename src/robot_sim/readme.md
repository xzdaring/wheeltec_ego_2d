```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch robot_sim v550_sim.launch.py x:=1.0 y:=2.0
```
启动机器人仿真环境，设置初始位置为 (1.0, 2.0)。



```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

## 二维激光仿真

`urdf/v550_sim.urdf.xacro` 在 `laser` link 上添加了 Gazebo ray 传感器，
通过 `libgazebo_ros_ray_sensor.so` 发布 `/scan`（`sensor_msgs/msg/LaserScan`），
`frame_id` 为 `laser`。仿真参数为 10 Hz、360 度、720 束、0.10–12 m，
用于功能验证，不代表实物雷达规格。世界物体必须有 collision 才能被检测。

修改源码后重新构建，并关闭旧仿真、重新生成机器人：

```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
colcon build --packages-select robot_sim
source install/setup.zsh
ros2 launch robot_sim v550_sim.launch.py
```

在另一个已加载 ROS 环境的终端分别检查（持续命令按 Ctrl+C 结束）：

```zsh
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --qos-reliability best_effort --once
ros2 run tf2_ros tf2_echo base_link laser
```

RViz 设置 Fixed Frame 为 `odom`，添加 LaserScan，Topic 选择 `/scan`，
Reliability 选择 Best Effort；建议 RViz 设置 use_sim_time=true。
应看到与墙壁位置一致的扫描点；机器人旋转后墙壁在 odom 中应保持原位。
无障碍物方向可能显示 inf，这是未命中量程内物体的正常结果。

这一步仅完成雷达仿真；EGO 不直接消费 LaserScan，后续仍需点云和坐标转换适配。
