# 底盘传感器可视化面板

这个面板用于在 149 机器人上只读查看底盘传感器：

- 激光雷达 `/slamware_ros_sdk_server_node/scan`
- SLAM 地图 `/slamware_ros_sdk_server_node/map`
- 里程计 `/slamware_ros_sdk_server_node/odom`
- 超声波/碰撞基础传感器 `/slamware_ros_sdk_server_node/basic_sensors_values`

服务默认监听 `0.0.0.0:18083`。机器人和本地电脑在同一网络时，可以直接通过机器人 IP 访问。

## 启动

在机器人上进入仓库：

```bash
cd ~/SLAMTEC_BASE_VISUALIZATION
nohup bash scripts/base_sensor_visual_server.sh > /tmp/base_sensor_visual_server_18083.log 2>&1 &
```

检查服务：

```bash
curl -s http://127.0.0.1:18083/api/health
```

## 开机启动

安装 systemd 服务：

```bash
cd ~/SLAMTEC_BASE_VISUALIZATION
sudo cp systemd/slamtec-base-visual.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now slamtec-base-visual.service
```

查看状态：

```bash
systemctl status slamtec-base-visual.service --no-pager
```

查看日志：

```bash
journalctl -u slamtec-base-visual.service -f
```

## 本地浏览器查看

直接打开：

```text
http://192.168.0.149:18083/
```

如果网络不方便直接访问，也可以在本地电脑做 SSH 转发：


```powershell
ssh -L 18083:127.0.0.1:18083 unitree@192.168.0.149
```

然后打开：

```text
http://127.0.0.1:18083/
```

## 页面内容

- `SLAM Map + Odometry`：显示 5cm/像素的占据栅格地图、机器人当前位置和轨迹。
- `Laser Scan`：显示激光雷达 2D 扫描点、最小距离、有效点数量。
- `Ultrasonic / Bumper Sensors`：显示两个 SONAR 和两个 BUMPER 的 id、安装位置、触发状态和值。
- `3D Point Cloud`：显示 `PointCloud2` 点云。当前默认 topic 是 `/ele_clouds`，这是底盘处理后的稀疏点云，不是深度相机原始稠密点云。
- `Raw State`：显示当前 API 状态摘要。

`3D Point Cloud` 面板可以用鼠标拖拽旋转视角。右侧读数里：

- `Topic`：当前使用的点云 topic
- `Points`：页面绘制点数 / 原始点数
- `Frame`：点云坐标系
- `Age`：距离上一帧的时间

## API

健康检查：

```bash
curl -s http://127.0.0.1:18083/api/health
```

完整状态：

```bash
curl -s http://127.0.0.1:18083/api/state
```

如果要临时改点云来源，可以手动启动时指定：

```bash
bash scripts/base_sensor_visual_server.sh --pointcloud-topic /ele_clouds
```

后续如果底盘深度相机服务能吐出稠密点云，比如 `/camera/depth/points`，可以改成：

```bash
bash scripts/base_sensor_visual_server.sh --pointcloud-topic /camera/depth/points
```

## 停止

```bash
pkill -f 'scripts/base_sensor_visual_server.py'
```
