# 思岚底盘传感器可视化

这是一个独立的小服务，用于查看 Unitree G1D 机器人底盘侧的思岚/SLAMWARE 传感器数据，并在 SLAM 地图上点选航点后交给 Slamware 底层导航。

当前支持：

- 激光雷达扫描
- SLAM 地图和里程计
- 超声波/碰撞传感器
- `/ele_clouds` 稀疏点云
- 当前 ROS 状态摘要
- 地图点选航点、路径预览、开始/停止 Slamware 导航

服务默认监听 `0.0.0.0:18083`，机器人和本地电脑在同一网络时，可以直接通过机器人 IP 打开网页。

## 启动

在机器人上进入部署目录：

```bash
cd ~/G1D_SLAM
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

## 地图选点导航

打开页面后，在 `SLAM Map + Odometry` 地图上点击添加航点：

- 紫色点/虚线：网页里手动选择的航点和直连预览。
- 绿色箭头：终点目标朝向，也就是发给 Slamware 的 `yaw`。
- 橙色线：Slamware 底层返回的 `global_plan_path`，也就是真实规划路径。
- `设置朝向`：先选好航点，再点击该按钮，然后在地图上点一下终点需要面向的方向。
- `清除朝向`：删除手动朝向，恢复为自动朝向。
- `角度°` + `应用角度`：直接输入终点目标角度，单位是度；例如 `90` 表示 90 度。
- `开始导航`：把当前航点发布到 `/slamware_ros_sdk_server_node/move_to_locations`。
- `停止导航`：发布 `/slamware_ros_sdk_server_node/cancel_action`。

如果没有手动设置朝向，页面会自动用最后一段路径方向计算终点 `yaw`。如果只有一个航点且没有手动朝向，后端会使用当前里程计 yaw。
服务发送导航请求时会同时设置 `MoveOptionFlag.WITH_YAW`，否则 Slamware 可能会忽略 `yaw` 字段并按路径方向结束。

开始导航前会做基础安全检查：地图、里程计、激光、超声/碰撞传感器需要新鲜；航点不能落在地图外或占用栅格上；碰撞/超声触发时不允许启动。当前 `localization_quality=0` 会显示警告，但默认不拦截，可用 `--min-localization-quality` 改成强制拦截。

## 开机启动

安装 systemd 服务：

```bash
cd ~/G1D_SLAM
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
systemd/slamtec-base-visual.service    开机启动服务
docs/base_sensor_visualization.md      中文使用说明
```

## 说明

`/ele_clouds` 当前只显示底盘集成层发布的稀疏 `PointCloud2` 点云，不是深度相机原始稠密点云。如果后续拿到原始深度点云 topic，可以用 `--pointcloud-topic` 切换。

详细说明见：

- `docs/base_sensor_visualization.md`

## 导航 API

开始导航：

```bash
curl -s -X POST http://127.0.0.1:18083/api/navigation/start \
  -H 'Content-Type: application/json' \
  -d '{"waypoints":[{"x":1.0,"y":2.0},{"x":1.5,"y":2.5}],"yaw_deg":90}'
```

`yaw` 也可以用弧度传：

```bash
curl -s -X POST http://127.0.0.1:18083/api/navigation/start \
  -H 'Content-Type: application/json' \
  -d '{"waypoints":[{"x":1.0,"y":2.0}],"yaw":1.5708,"yaw_source":"manual"}'
```

只想看会发送什么、不让机器人动，可以加 `dry_run`：

```bash
curl -s -X POST http://127.0.0.1:18083/api/navigation/start \
  -H 'Content-Type: application/json' \
  -d '{"waypoints":[{"x":1.0,"y":2.0}],"yaw_deg":90,"dry_run":true}'
```

停止导航：

```bash
curl -s -X POST http://127.0.0.1:18083/api/navigation/cancel -d '{}'
```

## 点位库 / 动作预留

18083 页面支持保存导航点位：

- `记录当前位置`：用当前 odom 的 `x/y/yaw` 生成一个点位。
- `保存点位`：手动编辑点位名称、`x/y/yaw_deg`、备注和动作 JSON。
- `加入导航`：把当前编辑点位加入页面上的航点队列，并把点位朝向作为终点 yaw；不会自动开始导航。
- `动作 JSON`：当前只保存，不执行，后续接机械臂动作时复用。

默认保存文件是 `data/nav_points.json`，启动时可用 `--points-file` 改路径。

## 动作链

`SLAM Map + Odometry` 右侧会显示当前动作链：

- 点击 `增加动作` 可以把动作加入链表；动作类型当前有 `导航` 和 `拾取熊猫烟`。
- `导航` 动作需要 `x/y/yaw_deg` 位姿；也可以直接在地图上点击快速增加导航动作。
- 动作卡片支持拖拽排序，也可以单独删除。
- `开始导航` 会抽取动作链里的所有导航动作执行，同时右侧显示导航进度。
- `执行动作链` 会按卡片顺序逐个执行：导航动作先到位，其他动作再执行。
- `拾取熊猫烟` 是假动作，后端会休眠 5 秒再返回。

点位 API：

```bash
curl -s http://127.0.0.1:18083/api/points

curl -s -X POST http://127.0.0.1:18083/api/points/record_current \
  -H 'Content-Type: application/json' \
  -d '{"name":"货架A等待点","actions":[]}'

curl -s -X POST http://127.0.0.1:18083/api/points/upsert \
  -H 'Content-Type: application/json' \
  -d '{"name":"手动点","x":1.2,"y":0.8,"yaw_deg":90,"actions":[]}'
```

假动作 API：

```bash
curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"fake_pick_xiongmao","name":"拾取熊猫烟","duration_sec":5}'
```
