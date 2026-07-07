# G1D SLAM 后端接口说明（新增功能）

本文档只详细说明**本轮新增**的后端接口，分三块：

1. [底盘遥控 Control / Jog](#一底盘遥控-control--jog)
2. [左眼相机 Camera](#二左眼相机-camera)
3. [2D 重定位 Reloc2D（含定时位姿保存）](#三2d-重定位-reloc2d含定时位姿保存)

文末附[全部后端接口简要汇总](#附录全部后端接口简要汇总)。

---

## 通用约定

- **服务地址**：后端 `http://<host>:18090`。开发模式下前端跑在 `:18089`，由 Vite 把 `/api/*` 代理到 `:18090`；生产模式由后端同端口托管前端静态页 + API。
- **响应格式**：统一 `application/json`（`StrictJSONResponse`，紧凑分隔符、`allow_nan=false`）。
- **通用返回结构**：多数接口返回 `{ "ok": true, ... }`；失败返回 `{ "ok": false, "error": "<原因>" }`。
- **HTTP 状态码**：`400` 请求体非法 / `413` 请求体过大（>64KiB）/ `500` 服务端异常 / `503` 依赖不可用（如相机流未就绪）。
- **请求体**：POST 一律 JSON object。

---

## 一、底盘遥控 Control / Jog

用于**建图采集界面**的手动遥控（前进/后退/左转/右转）。底层调用 unitree SDK 的 `g1d_simple_control`（与独立的 :18086 服务同一条已验证链路），**不走 ROS `/cmd_vel`**（实测该话题无法驱动本底盘）。

**工作机制**
- 「按住持续动」：前端在按住方向键期间每 ~300ms 发一次 `POST /api/control/jog`（心跳）；相同动作只刷新心跳、不重启进程，保证运动连续。松开发 `POST /api/control/stop`。
- **服务端死人开关（deadman）**：后端看门狗每 200ms 检查，若运动中超过 `deadman_timeout_sec`（默认 1s）没收到新心跳（关标签页/断网/卡顿），自动停车。这是主要安全网。
- **速度钳制**：线速/角速分别钳到上限（默认线速 ≤0.25 m/s、角速 ≤0.6 rad/s）。

### 1. `POST /api/control/jog`
启动或刷新一次「按住」运动。

请求字段：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `action` | string | 是 | 动作：`forward`/`back`/`turn_left`/`turn_right`（也接受别名 `up`/`down`/`left`/`right`） |
| `speed` | number | 否 | 速度；线动作单位 m/s，转向单位 rad/s。缺省用后端默认值；超上限自动钳制 |

响应字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `ok` | bool | 是否成功 |
| `mode` | string | `started`（新起进程）/ `heartbeat`（刷新心跳）|
| `action` | string | 归一化后的动作 |
| `speed` | number | 实际生效速度（已钳制）|
| `pid` | number | 控制进程 pid（仅 `started` 时）|
| `error` | string | 失败原因（如动作非法、二进制不存在）|

示例：
```bash
curl -X POST http://127.0.0.1:18090/api/control/jog \
  -H 'Content-Type: application/json' \
  -d '{"action":"forward","speed":0.15}'
```

### 2. `POST /api/control/stop`
停止当前遥控（终止控制进程 + 发一次 `stop` + 清理残留进程）。无请求体。

响应：`{ "ok": true, "mode": "stopped", "terminated": {...}, ... }`

### 3. `GET /api/control/status`
查询遥控状态与限幅参数（前端用它初始化滑条上限）。

响应字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `active` | bool | 当前是否有运动进程 |
| `action` / `speed` | string / number | 当前动作与速度 |
| `binary_ok` | bool | 控制二进制是否存在可用 |
| `interface` | string | 网口（如 `eth0`）|
| `max_linear_mps` / `max_angular_radps` | number | 线速/角速上限 |
| `default_linear_mps` / `default_angular_radps` | number | 线速/角速默认值 |
| `deadman_timeout_sec` | number | 死人开关超时 |

**相关配置项**（`run.sh`/启动参数）：`--base-control-bin`、`--jog-max-linear-mps`、`--jog-max-angular-radps`、`--jog-default-linear-mps`、`--jog-default-angular-radps`、`--jog-hold-duration-sec`、`--jog-deadman-timeout-sec`。

---

## 二、左眼相机 Camera

在建图采集界面显示机器人**左眼**画面。后端通过 ZMQ 订阅头相机 JPEG 流（与 teleimager/YOLO 同一数据源），裁出并排双目图的**左半**，重新编码为浏览器 `<img>` 可直接显示的 **MJPEG** 流。

**工作机制**
- ZMQ 端口自动发现：向本机相机配置服务（`REP:60000`）请求 `head_camera.zmq_port`（当前为 `55555`，图像 1280×480 并排双目）。
- **引用计数**：MJPEG 首个客户端连接时才启动 SUB 线程；最后一个断开后短暂 linger 再停，无人观看时不占用相机。
- 依赖 `cv2` + `pyzmq`；启动脚本用 `LD_PRELOAD=libgomp` 规避 aarch64 的 TLS 报错。

### 1. `GET /api/camera/left_eye`
左眼 **MJPEG 流**（`multipart/x-mixed-replace; boundary=frame`）。直接用于 `<img :src="/api/camera/left_eye">`。不可用时返回 `503`。

### 2. `GET /api/camera/left_eye.jpg`
左眼**单帧快照**（`image/jpeg`）。无帧时 `503`，解码失败 `500`。

### 3. `GET /api/camera/status`
相机流状态。

响应字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `active` | bool | SUB 线程是否在运行 |
| `clients` | number | 当前 MJPEG 客户端数 |
| `host` | string | 相机源地址 |
| `resolved_zmq_port` | number | 实际订阅的 ZMQ 端口 |
| `has_frame` | bool | 是否已有帧 |
| `frame_age_sec` | number | 最近一帧的年龄（秒）|
| `eye` | string | `left` / `right` |
| `error` | string\|null | 最近错误（如发现配置失败）|

**相关配置项**：`--head-camera-host`、`--head-camera-request-port`、`--head-camera-zmq-port`（0=自动发现）、`--head-camera-jpeg-quality`、`--head-camera-max-fps`、`--head-camera-eye`。

---

## 三、2D 重定位 Reloc2D（含定时位姿保存）

基于 **2D 激光 ↔ 占据栅格**配准的重定位（纯 `numpy`/`scipy`），提供三种方式；算出的位姿可选择「应用到底盘」（复用 `set_pose` + `recover_localization`）。同时提供一个**定时把当前有效位姿保存到 `last_pose.json`** 的机制，供「上次位姿」初值使用。

> 有效性保护：定时保存仅在定位真正有效时执行（`is_localization_enabled` 且 `localization_quality>0` 且状态新鲜），避免开机未定位时把未知位姿写入、污染 json 初值。

### 1. `POST /api/reloc2d/run`
运行一次 2D 重定位。

请求字段：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `method` | string | 是 | `global`（全图搜索，无需初值）/ `json`（以 `last_pose.json` 为初值的双阶段 ICP）/ `click`（以手动点击点为初值）|
| `init` | object | `click` 必填 | `{x, y, yaw?}`；`click` 用（yaw 可省，由算法搜索）|
| `apply` | bool | 否 | `true` 则把结果 `set_pose` 喂给底盘。前端默认 `false`（先预览再单独应用）|
| `params` | object | 否 | 透传给匹配器的可选参数 |
| `rmse_accept` | number | 否 | 判定「达标」的 RMSE 阈值（默认 0.15 m）|

响应字段（成功时）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `ok` | bool | 是否成功 |
| `pose` | object | 结果位姿 `{x, y, yaw, yaw_deg}` |
| `accepted` | bool | 是否达标（RMSE ≤ 阈值）|
| `rmse` | number\|null | 配准 RMSE（m）|
| `fitness` | number | 内点占比 |
| `spread` | object | 多起点散布 `{pos_rms, yaw_std_deg}`（可信度参考）|
| `n_starts` | number | 起点数 |
| `n_scan` / `n_map` | number | 参与匹配的扫描点数 / 地图点数 |
| `input_method` | string | 回显 `method` |
| `init` | object\|null | 实际使用的初值 |
| `applied` | bool | 是否已应用到底盘 |
| `apply_result` | object | `apply=true` 时的应用结果 |

失败示例：`{"ok":false,"error":"没有可用的 last_pose.json（等待定时保存或先跑一会儿）"}`

示例：
```bash
curl -X POST http://127.0.0.1:18090/api/reloc2d/run \
  -H 'Content-Type: application/json' \
  -d '{"method":"click","init":{"x":1.2,"y":-3.4},"apply":false}'
```

### 2. `GET /api/reloc2d/last_pose`
读取最近保存的位姿。响应：`{ "ok": bool, "last_pose": {x,y,z,yaw,yaw_deg,frame_id,saved_at} | null }`

### 3. `GET /api/reloc2d/config`
读取定时保存配置。

响应字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `interval_sec` | number | 当前保存间隔（秒）|
| `enabled` | bool | 是否启用（间隔>0）|
| `path` | string | json 文件路径 |
| `last_pose` | object\|null | 最近保存的位姿 |

### 4. `POST /api/reloc2d/config`
设置定时保存间隔（运行时可调）。

请求字段：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `interval_sec` | number | 是 | 保存间隔秒数；`<=0` 关闭自动保存，`>0` 钳到 [1, 3600] |

响应：同 `GET /api/reloc2d/config`（附 `ok`）。

**相关配置项**：`--last-pose-file`、`--last-pose-save-interval-sec`。

---

## 附录：全部后端接口简要汇总

★ = 本轮新增（上文已详述）。其余为原单体服务迁移而来。

| 分组 | 方法 | 路径 | 用途 |
|---|---|---|---|
| 状态 | GET | `/api/state` | 全量遥测快照（scan/map/odom/track/sensors/navigation…）|
| 状态 | GET | `/api/health` | 存活检查 |
| 导航点 | GET | `/api/points` | 列出已保存导航点 |
| 导航点 | POST | `/api/points/upsert` `/delete` `/record_current` | 增改 / 删除 / 记录当前点 |
| 导航 | POST | `/api/nav/start` `/stop` `/cancel`（`/api/navigation/*` 同）| 启动 / 停止 / 取消导航（含 raw cmd_vel 无避障）|
| 建图 | GET | `/api/mapping/status` `/list`(`/files`) | 建图状态 / 已存档地图列表 |
| 建图 | POST | `/api/mapping/start` `/stop` `/save` `/load` | 开始 / 结束 / 保存 / 加载地图 |
| 重定位·锚点 | GET | `/api/relocalization/status` | 锚点重定位状态 |
| 重定位·锚点 | POST | `/api/relocalization/run` `/save_anchor` `/start` | 运行 / 保存锚点 / 启动 |
| 重定位·2D ★ | POST | `/api/reloc2d/run` | 2D 激光重定位（global/json/click）|
| 重定位·2D ★ | GET | `/api/reloc2d/last_pose` | 读取最近保存位姿 |
| 重定位·2D ★ | GET/POST | `/api/reloc2d/config` | 读取 / 设置定时保存间隔 |
| 底盘遥控 ★ | POST | `/api/control/jog` `/stop` | 遥控点动（按住心跳）/ 停止 |
| 底盘遥控 ★ | GET | `/api/control/status` | 遥控状态与限幅 |
| 相机 ★ | GET | `/api/camera/left_eye` | 左眼 MJPEG 流 |
| 相机 ★ | GET | `/api/camera/left_eye.jpg` | 左眼单帧快照 |
| 相机 ★ | GET | `/api/camera/status` | 相机流状态 |
| 动作 | POST | `/api/actions/execute` `/stop_all`(`/stop`) | 执行动作（机械臂/升降柱等）/ 全部停止 |
| 升降柱 | GET | `/api/lift_height`(`/column_height/current`) | 读取升降柱高度 |
| 故障 | GET | `/api/fault_snapshots` `/log` | 故障快照列表 / 日志 |
| 故障 | POST | `/api/fault_snapshots/clear` | 清空故障快照 |

---

### 启动方式（参考）

```bash
./start.sh          # 生产:构建前端 + 后端(:18090) 同端口托管
./start.sh dev      # 开发:后端(:18090) + Vite(:18089) 并行,热更新
```

`backend/run.sh` 会 source ROS Foxy + slamware 环境，用系统 `python3.8`（因 rclpy），并 `LD_PRELOAD` libgomp（供相机的 cv2/pyzmq 使用）。
