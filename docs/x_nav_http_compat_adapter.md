# x_nav 零改动兼容适配层

这个适配层用于让作业平台不改代码、不改端口、不改接口，继续按公司 x_nav 文档里的 `9000/api/extra/*` 或 ROS1 topic 调用导航系统。机器人侧再把这些调用转换到本仓库现有的 `18083` G1D_SLAM/Slamware 服务。

核心原则：

- 平台继续访问 `http://机器人IP:9000`。
- 已知接口尽量返回 x_nav 风格结构。
- 未知 HTTP 接口不直接 404，而是返回结构化 `success:false` 并写入日志。
- 现场跑一次平台后，通过日志补齐真实使用到的接口。

## 启动顺序

先启动现有底盘服务：

```bash
cd ~/G1D_SLAM
nohup bash scripts/base_sensor_visual_server.sh > /tmp/base_sensor_visual_server_18083.log 2>&1 &
curl -s http://127.0.0.1:18083/api/health
```

再启动兼容适配层：

```bash
cd ~/G1D_SLAM
nohup bash scripts/x_nav_http_compat_adapter.sh > /tmp/x_nav_http_compat_adapter_9000.log 2>&1 &
curl -s http://127.0.0.1:9000/api/health
curl -s http://127.0.0.1:9000/api/compat/capabilities
```

如果 `18083` 不在本机，可以指定上游地址：

```bash
bash scripts/x_nav_http_compat_adapter.sh --upstream http://192.168.0.149:18083
```

## 盲联调日志

默认所有请求都会写入：

```text
data/x_nav_compat_requests.jsonl
```

现场联调时，可以直接看最近请求：

```bash
tail -f data/x_nav_compat_requests.jsonl
curl -s 'http://127.0.0.1:9000/api/compat/request_log/tail?limit=50'
```

如果作业平台调用了未实现接口，日志里会出现：

```json
{"unknown_endpoint":true,"method":"POST","path":"/api/extra/xxx",...}
```

这就是后续补齐兼容面的依据。

## 机器人侧避障模式开关

作业平台不需要改请求。平台仍然按 x_nav 的 `nav_custom` 下发目标点，机器人侧可以通过兼容层开关决定默认导航模式。

查看当前模式：

```bash
curl http://127.0.0.1:9000/api/compat/config
```

普通避障模式（默认）：

```bash
curl -X POST http://127.0.0.1:9000/api/compat/config \
  -H "Content-Type: application/json" \
  -d '{"default_navigation_mode":"normal"}'
```

直连不绕障，仍走 Slamware `move_to_locations`，遇障停止：

```bash
curl -X POST http://127.0.0.1:9000/api/compat/config \
  -H "Content-Type: application/json" \
  -d '{"default_navigation_mode":"direct_no_avoidance"}'
```

裸控不避障，兼容层会让 18083 直接发布 `/cmd_vel`：

```bash
curl -X POST http://127.0.0.1:9000/api/compat/config \
  -H "Content-Type: application/json" \
  -d '{"default_navigation_mode":"raw_cmd_vel_no_obstacle_avoidance"}'
```

裸控速度默认与 18083 网页端一致，可调整：

```bash
curl -X POST http://127.0.0.1:9000/api/compat/config \
  -H "Content-Type: application/json" \
  -d '{"default_navigation_mode":"raw_cmd_vel_no_obstacle_avoidance","raw_linear_speed_mps":0.35,"raw_angular_speed_radps":1.2}'
```

如果请求里显式带了 `raw_cmd_vel`、`direct_no_avoidance` 或 `navigation_mode`，显式字段优先；否则使用机器人侧默认开关。

## 已映射接口

| x_nav 文档接口 | 当前映射 |
|---|---|
| `GET /api/extra/get_status` | 读取 `GET /api/state`，转成 x_nav 风格状态字段 |
| `GET /api/extra/get_nav_state` | 返回兼容推导的 `/x_nav/state`、`/x_nav/slam/state`、`/x_nav/master/state` |
| `GET /api/compat/config` | 查看机器人侧默认导航模式 |
| `POST /api/compat/config` | 设置机器人侧默认导航模式 |
| `GET /api/extra/get_map_data?page=1` | 读取 `GET /api/mapping/list` |
| `GET /api/extra/get_nav_data?page=1` | 读取 `GET /api/points` |
| `GET /api/extra/get_obstacle_data?page=1` | 返回空列表 |
| `GET /api/extra/refresh_point_cloud` | 健康检查式 no-op |
| `GET /api/extra/nav_work/cancel?stop=1` | 转发到 `POST /api/navigation/cancel` |
| `GET /api/extra/nav_work/cancel` | 兼容 no-op，返回继续导航成功 |
| `POST /api/extra/add_map` | 转发到 `POST /api/mapping/start` |
| `POST /api/extra/save_map` | 转发到 `POST /api/mapping/save` |
| `POST /api/extra/switch_map` | 转发到 `POST /api/mapping/load` |
| `POST /api/extra/nav_custom` | 转发到 `POST /api/navigation/start` |
| `POST /api/compat/navigation/start` | 给 ROS1 shim 使用，直接转发多点导航 |
| `POST /api/extra/mark_nav_point` | 转发到 `POST /api/points/upsert` |
| `POST /api/extra/set_pose` | 转发到 `POST /api/relocalization/run` |

## x_nav 状态兼容

适配层会维护一份 x_nav 风格状态：

- `-1`：空闲，无导航任务
- `-2`：载入地图初始化成功
- `0`：执行导航任务中
- `12`：即将到达目标点，正在对齐
- `13`：到达目标点
- `14`：接收到导航目标点，路径规划成功
- `15`：路径规划失败
- `16`：无 odom，无法导航

状态推导来自 G1D_SLAM 的 `last_command`、odom、地图状态和定位开关。它不是完整 x_nav 内部算法状态，但足够先支撑平台任务流判断。到达阈值可启动时调整：

```bash
bash scripts/x_nav_http_compat_adapter.sh \
  --arrival-distance-m 0.18 \
  --arrival-yaw-deg 4.0 \
  --odom-stale-sec 2.0
```

## 可选 ROS1 shim

如果平台直接使用 ROS1 topic，启动 HTTP 兼容层后，再在 ROS1 环境里启动：

```bash
cd ~/G1D_SLAM
python3 scripts/x_nav_ros1_compat_node.py _compat_base_url:=http://127.0.0.1:9000
```

当前 ROS1 shim 支持：

- 订阅 `/move_base_simple/goal`，转成 `POST /api/extra/nav_custom`
- 订阅 `/initialpose`，转成 `POST /api/extra/set_pose`
- 订阅 `/planner/cmd`，支持 `cancel_nav`，其它常见 planner 指令先作为兼容 no-op
- 订阅 `/node_cmd`，支持 `launch_mapping#map`、`save_map#map`、`launch_navigation#map`
- 订阅 `/topological_path`，转成 `POST /api/compat/navigation/start`
- 发布 `/x_nav/state`
- 发布 `/x_nav/slam/state`
- 发布 `/x_nav/master/state`

暂未实现 ROS1 custom service：`/x_nav/planner/service`、`/x_nav/slam/service`。如果平台日志确认使用了这两个 service，需要拿到 x_nav 的 `.srv` 定义后补。

## 暂未实现

- 完整 ROS1 service 兼容：`/x_nav/planner/service`、`/x_nav/slam/service` 暂未实现。
- 虚拟障碍物：当前 G1D_SLAM/Slamware 兼容层没有等价的 x_nav MarkerArray 障碍物写入接口。
- `web_cmd` 动作：`/api/extra/target_angle?angle=N` 暂不映射到底盘步态或机器人动作。
- 真正暂停/继续导航：`nav_work/cancel` 无 `stop=1` 时只做兼容 no-op。

## 联调建议

1. 先启动 `18083` 和 `9000`，让平台原样连接机器人。
2. 同时 `tail -f data/x_nav_compat_requests.jsonl`。
3. 跑平台最小任务：加载地图、重定位、单点导航、停止。
4. 如果日志没有未知接口，继续跑完整分拣流程。
5. 如果出现未知 HTTP 接口，根据日志补。
6. 如果发现平台使用 ROS1 topic，启动 `x_nav_ros1_compat_node.py`。
7. 如果发现平台使用 ROS1 service，补 x_nav `.srv` 后再实现 service。
8. 导航安全验证时优先使用现有 `18083` 页面确认地图、odom、laser、bumper/sonar 都是新鲜数据。
## 前端控制台

兼容层内置了一个轻量控制台，不需要额外起前端服务。启动 `9000` adapter 后，在同一网络下打开：

```text
http://机器人IP:9000/
```

也可以访问：

```text
http://机器人IP:9000/api/compat/control
```

页面可以查看当前机器人侧默认导航模式，并直接切换：

- `normal`：普通避障，作业平台请求保持原样。
- `direct_no_avoidance`：直连少绕路，仍走底层导航。
- `raw_cmd_vel_no_obstacle_avoidance`：裸控不避障，adapter 直接让 `18083` 走 `/cmd_vel` 逻辑。

页面还提供裸控线速度/角速度参数、停止导航按钮、`9000` 健康状态、x_nav 状态和最近请求日志。作业平台仍然只需要按原 x_nav 接口调用 `9000/api/extra/*`，不需要改请求体。
