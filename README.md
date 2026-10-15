# wheeltec_ego_2d
把ego_planner算法降维为2d部署到轮趣科技的v550_mec上

## V550_mec 实车（N10P）

参见 [实车操作说明](src/robot_sim/REAL_ROBOT.md)。硬件入口 `v550_real_bringup.launch.py`，导航入口 `ego_real.launch.py`。
导航默认预览速度，确认接口与轨迹后用 `enable_motion:=true` 接入底盘。
