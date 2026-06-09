# 思岚底盘传感器可视化

这是一个独立的小服务，用于只读查看 Unitree G1D 机器人底盘侧的思岚/SLAMWARE 传感器数据。

当前支持：

- 激光雷达扫描
- SLAM 地图和里程计
- 超声波/碰撞传感器
- `/ele_clouds` 稀疏点云
- 当前 ROS 状态摘要

服务默认监听 `0.0.0.0:18083`，机器人和本地电脑在同一网络时，可以直接通过机器人 IP 打开网页。

## 启动

在机器人上进入部署目录：

```bash
cd ~/unifolm-world-model-action/robot_client_unitree_g1_full_20260509/repos/unitree_deploy
nohup bash scripts/base_sensor_visual_server.sh > /tmp/base_sensor_visual_server_18083.log 2>&1 &
```

检查服务：

```bash
curl -s http://127.0.0.1:18083/api/health
```

本地电脑直接打开：

```text
http://192.168.0.149:18083/
```

## 依赖

机器人侧需要已有 ROS2 Foxy 和思岚底盘服务环境：

```bash
source /opt/ros/foxy/setup.bash
source /unitree/module/slamware_service_pc4/install/setup.bash
```

启动脚本 `scripts/base_sensor_visual_server.sh` 已经包含这两个 `source`。

## 文件

```text
scripts/base_sensor_visual_server.py   Web + ROS2 可视化服务
scripts/base_sensor_visual_server.sh   机器人启动脚本
docs/base_sensor_visualization.md      中文使用说明
```

## 说明

`/ele_clouds` 当前只显示底盘集成层发布的稀疏 `PointCloud2` 点云，不是深度相机原始稠密点云。如果后续拿到原始深度点云 topic，可以用 `--pointcloud-topic` 切换。

详细说明见：

- `docs/base_sensor_visualization.md`
