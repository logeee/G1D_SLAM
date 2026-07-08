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

立柱高度常驻服务默认监听 `0.0.0.0:28089`，给 18083 页面和外部前端提供当前立柱真实物理高度：

```bash
cd ~/G1D_SLAM
nohup bash scripts/g1d_lift_height_service.sh > /tmp/g1d_lift_height_service_28089.log 2>&1 &
curl -s http://127.0.0.1:28089/api/basic_status
```

18083 后端默认读取 `http://127.0.0.1:28089/api/basic_status`。如果这个服务没有启动，页面会显示“当前立柱高度：读取失败 / Connection refused”。

## 开机启动

安装 systemd 服务：

```bash
cd ~/G1D_SLAM
sudo cp systemd/g1d-lift-height.service /etc/systemd/system/
sudo cp systemd/slamtec-base-visual.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now g1d-lift-height.service
sudo systemctl enable --now slamtec-base-visual.service
```

查看状态：

```bash
systemctl status g1d-lift-height.service --no-pager
systemctl status slamtec-base-visual.service --no-pager
```

查看日志：

```bash
journalctl -u g1d-lift-height.service -f
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
- `直连不绕障`：使用 Slamware `MoveOptionFlag.KeyPoints` 模式，按指定航点直连走，不做自动绕障；遇到障碍会停止。
- `裸控无避障`：绕开 Slamware 导航，服务根据 odom 低速闭环发布 `/cmd_vel`，不会自动避障；只在确认路径完全安全时使用。
- `停止`：发送 `/slamware_ros_sdk_server_node/cancel_action`。

如果没有手动设置朝向，页面会自动用最后一段路径方向计算终点 `yaw`。如果只有一个航点且没有手动朝向，后端会使用当前里程计 yaw。
服务发送导航请求时会同时设置 `MoveOptionFlag.WITH_YAW`，否则 Slamware 可能会忽略 `yaw` 字段并按路径方向结束。勾选 `直连不绕障` 时会额外设置 `MoveOptionFlag.KeyPoints`。
勾选 `裸控无避障` 时不会发布 `/move_to_locations`，而是直接发布 `/cmd_vel`；`停止` 会同时取消 Slamware 动作并发送零速度。

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

- 点击 `增加动作` 可以把动作加入链表；动作类型当前有 `导航`、`机械臂抓取`、`机械臂放置`、`机械臂复位`、`立柱升降`。
- `导航` 动作可以直接选择点位库里的点位，自动填入 `x/y/yaw_deg`；也可以手动填写，或直接在地图上点击快速增加导航动作。
- `机械臂抓取` 会选择目标标签并发布 ROS JSON 到 `/arm_control/task_command`，目前目标标签为 `XiongMao` 和 `Xizi_Liqun`。
- `机械臂放置` / `机械臂复位` 会发布 `PLACE` / `RESET`，`target_object` 留空。
- `立柱升降` 是原始动作，默认调用 `/home/unitree/unitree_sdk2/build/bin/g1d_height_control eth0 <目标高度m>`。
- 动作卡片可以拖拽排序，也可以单独删除。
- `清空动作` 会清空整个动作链，包括导航动作和机械臂动作。
- `执行动作链` 会按卡片顺序逐个执行：导航动作先到位，其他动作再执行。导航动作完成条件是距离目标点 180mm 内、目标 yaw 偏差 4 度内，并连续稳定约 1.2 秒。
- `仅执行导航` 只用于调试导航，会跳过非导航动作。
- `停止` 会取消 Slamware 导航，并向机械臂发布停止/复位 phase。默认发布 `RESET`、`SUCTION_STOP`、`MOTION_STOP`，可用 `--arm-stop-phases` 调整。
- 机械臂任务会等待 `/arm_control/task_status` 中同一个 `task_id` 的终态：`DONE` 成功，`FAILED` / `REJECTED` 失败，默认超时 120 秒。

机械臂任务接口：

```bash
curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"arm_task","phase":"PICK","target_object":"XiongMao","timeout_sec":120}'

curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"arm_task","phase":"PLACE","target_object":"","timeout_sec":120}'

curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"arm_task","phase":"RESET","target_object":"","timeout_sec":120}'
```

立柱升降原始动作接口：

```bash
curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"column_height","target_height_m":0.0,"timeout_sec":30}'

curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"column_height","target_height_m":0.247,"timeout_sec":30,"dry_run":true}'
```

只验证 JSON 不发布给机械臂时，可以加 `dry_run:true`：

```bash
curl -s -X POST http://127.0.0.1:18083/api/actions/execute \
  -H 'Content-Type: application/json' \
  -d '{"type":"arm_task","phase":"PICK","target_object":"Xizi_Liqun","dry_run":true}'
```

## 立柱高度服务 API

常驻服务：

```bash
systemctl status g1d-lift-height.service --no-pager
curl -s http://127.0.0.1:28089/api/basic_status
```

外部前端直接读取：

```text
GET http://<机器人IP>:28089/api/basic_status
GET http://<机器人IP>:28089/api/lift_height
```

主要字段：

- `physical_height_m`：当前立柱真实物理高度，单位 m，默认范围 `0.0 ~ 0.427`。
- `hispeed_y_m`：DDS `rt/hispeed_state` 的 raw y 值。
- `lift_offset_m`：raw 到物理高度的偏移。默认每次开机自动重新标定，优先使用本次 boot 的标定文件。
- `sdk_min_m` / `sdk_max_m`：raw 控制范围。本次 boot 标定后 `sdk_min_m` 等于开机自动降到底后的最低位 raw 值。
- `full_travel_m`：物理总行程，默认 `0.427`。
- `calibration_source`：`auto_boot_min_calibration` 表示使用本次开机自动标定，`manual_min_calibration` 表示人工标定，`stale_file_pending_auto_boot` 表示旧 boot 标定已失效、正在等待本次开机标定。
- `auto_calibration`：自动标定后台任务状态，例如 `waiting_for_boot_zero` / `sampling` / `done` / `skipped` / `failed`。

默认开机流程：服务启动后等待 `G1D_LIFT_AUTO_CALIBRATE_DELAY_SEC`（默认 75 秒），让 G1-D 完成自动降到底；随后采样 `hispeed_y_m`，稳定后写入本次 boot 的 offset。标定文件默认保存在 `/home/unitree/.config/g1d_lift_height/calibration.json`，并带有 `boot_id`；下次开机 `boot_id` 变化后旧 offset 不再当真值。

自动标定完成前，接口会返回 `offset_valid=false`，此时不要使用 `physical_height_m` 做控制。

如果现场已经确认立柱在最低位，也可以手动把当前最低位标定为物理 `0m`：

```bash
curl -s http://127.0.0.1:28089/api/calibrate_min
curl -s http://127.0.0.1:28089/api/basic_status
```

如果误标定，可以删除标定并等待自动标定或重新手动标定：

```bash
curl -s http://127.0.0.1:28089/api/reset_calibration
```

服务通过 `unitree_sdk2_python` 直接订阅 DDS，不依赖 `ros2 topic echo`。手动启动示例：

```bash
bash scripts/g1d_lift_height_service.sh \
  --dds-interface eth0 \
  --dds-hispeed-topic rt/hispeed_state \
  --full-travel-m 0.427 \
  --auto-calibrate-delay-sec 75 \
  --auto-calibrate-max-uptime-sec 300
```

机械臂模块 v2.1 的 ROS Topic 约定：

- 指令 Topic：`/arm_control/task_command`
- 状态 Topic：`/arm_control/task_status`
- 消息类型：`std_msgs/String`
- 指令 JSON 核心字段：`task_id` / `phase` / `target_object`
- `phase` 取值：`RESET` / `PICK` / `PLACE`
- `target_object`：`PICK` 必填，必须与 YOLO 标签对应；`PLACE` / `RESET` 留空
