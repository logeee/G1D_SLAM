# 底盘传感器可视化面板

这个面板用于在 149 机器人上查看底盘传感器，并在 SLAM 地图上点选航点后调用思岚/Slamware 底层导航：

- 激光雷达 `/slamware_ros_sdk_server_node/scan`
- SLAM 地图 `/slamware_ros_sdk_server_node/map`
- 里程计 `/slamware_ros_sdk_server_node/odom`
- 超声波/碰撞基础传感器 `/slamware_ros_sdk_server_node/basic_sensors_values`
- Slamware 规划路径 `/slamware_ros_sdk_server_node/global_plan_path`
- Slamware 导航请求 `/slamware_ros_sdk_server_node/move_to_locations`
- Slamware 停止请求 `/slamware_ros_sdk_server_node/cancel_action`

服务默认监听 `0.0.0.0:18083`。机器人和本地电脑在同一网络时，可以直接通过机器人 IP 访问。

## 启动

在机器人上进入仓库：

```bash
cd ~/G1D_SLAM
nohup bash scripts/base_sensor_visual_server.sh > /tmp/base_sensor_visual_server_18083.log 2>&1 &
```

检查服务：

```bash
curl -s http://127.0.0.1:18083/api/health
```

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

- `SLAM Map + Odometry`：显示 5cm/像素的占据栅格地图、机器人当前位置和轨迹，也可以点击地图添加航点。
- `Laser Scan`：显示激光雷达 2D 扫描点、最小距离、有效点数量。
- `Ultrasonic / Bumper Sensors`：显示两个 SONAR 和两个 BUMPER 的 id、安装位置、触发状态和值。
- `3D Point Cloud`：显示 `PointCloud2` 点云。当前默认 topic 是 `/ele_clouds`，这是底盘处理后的稀疏点云，不是深度相机原始稠密点云。
- `Raw State`：显示当前 API 状态摘要。

### 地图选点导航

在 `SLAM Map + Odometry` 地图上点击即可添加航点：

- 紫色点/虚线：网页手动选择的航点和直连预览。
- 绿色箭头：终点目标朝向，对应发给 Slamware 的 `yaw`。
- 橙色线：Slamware 底层发布的 `global_plan_path`。
- `撤销点`：删除最后一个航点。
- `清空点`：删除所有航点。
- `设置朝向`：先选好航点，再点击该按钮，然后在地图上点一下终点需要面向的方向。
- `清除朝向`：删除手动朝向，恢复为自动朝向。
- `角度°` + `应用角度`：直接输入终点目标角度，单位是度；例如 `90` 表示 90 度。
- 地图工具栏只负责构建导航动作；真正执行入口在右侧 `动作链` 面板。
- `执行动作链`：主执行入口，会按动作卡片顺序执行导航和后续动作。
- `仅执行导航`：调试入口，只抽取动作链里的导航动作发送到 `/slamware_ros_sdk_server_node/move_to_locations`。
- `停止`：发送 `/slamware_ros_sdk_server_node/cancel_action`。

如果没有手动设置朝向，页面会自动用最后一段路径方向计算终点 `yaw`。如果只有一个航点且没有手动朝向，后端会使用当前里程计 yaw。
服务发送导航请求时会同时设置 `MoveOptionFlag.WITH_YAW`，否则 Slamware 可能会忽略 `yaw` 字段并按路径方向结束。

开始导航前，服务会检查地图、里程计、激光、超声/碰撞传感器是否新鲜，并检查航点是否在地图内、是否落在占用栅格上。碰撞/超声触发时不会启动导航。当前 `localization_quality=0` 默认只提示警告；如需强制限制，可以启动时增加 `--min-localization-quality N`。

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

## 点位库 / 动作预留

页面里的 `点位库 / 动作预留` 用来保存可复用导航点位：

- `记录当前位置`：读取当前 `/slamware_ros_sdk_server_node/odom`，保存当前机器人 `x/y/yaw`。
- `保存点位`：手动新建或编辑点位，字段包括名称、`x`、`y`、`yaw_deg`、备注、动作 JSON。
- `加入导航`：把点位追加到当前页面的航点队列，并把点位 `yaw_deg` 同步成终点目标角度；不会自动开始导航。
- 动作 JSON 只是预留字段，当前不会调用机械臂，也不会执行任何动作。

默认数据文件：

```text
data/nav_points.json
```

启动时可以改路径：

```bash
bash scripts/base_sensor_visual_server.sh --points-file /home/unitree/G1D_SLAM/data/nav_points.json
```

常用 API：

```bash
# 查看点位列表
curl -s http://127.0.0.1:18083/api/points

# 保存当前机器人位置
curl -s -X POST http://127.0.0.1:18083/api/points/record_current \
  -H 'Content-Type: application/json' \
  -d '{"name":"货架A等待点","actions":[]}'

# 手动保存点位
curl -s -X POST http://127.0.0.1:18083/api/points/upsert \
  -H 'Content-Type: application/json' \
  -d '{"name":"手动点","x":1.2,"y":0.8,"yaw_deg":90,"note":"测试","actions":[]}'

# 删除点位
curl -s -X POST http://127.0.0.1:18083/api/points/delete \
  -H 'Content-Type: application/json' \
  -d '{"id":"pt_xxx"}'
```

## 动作链

`SLAM Map + Odometry` 右侧是动作链面板：

- 点击 `增加动作` 可以把动作加入链表；动作类型当前有 `导航` 和 `拾取熊猫烟`。
- `导航` 动作可以直接选择点位库里的点位，自动填入 `x/y/yaw_deg`；也可以手动填写，或直接在地图上点击快速增加导航动作。
- 动作卡片可以拖拽排序，也可以单独删除。
- `执行动作链` 会按卡片顺序逐个执行：导航动作先到位，其他动作再执行。
- `仅执行导航` 只用于调试导航，会跳过非导航动作。
- `拾取熊猫烟` 目前是假动作，后端会休眠 5 秒后返回成功。

假动作接口：

```bash
curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"fake_pick_xiongmao","name":"拾取熊猫烟","duration_sec":5}'
```

后续接机械臂时，可以保留同一个动作链 UI，把 `fake_pick_xiongmao` 替换成真实动作类型，或者在 `/api/actions/execute` 里根据 `type` 分发到机械臂 SDK。
