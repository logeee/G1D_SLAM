# G1D SLAM 前端（Vue3）

从 `scripts/base_sensor_visual_server.py` 内嵌的原生 HTML/CSS/JS 抽离并逐步迁移到 Vue3 + Vite。

## 与后端的关系

- 后端（`backend/base_sensor_visual_server.py`，从 `scripts/` 复制并抽离前端后的副本）只负责 ROS2 + REST API，端口 `18083`。
- 所有接口路径都是相对的（`/api/...`），因此：
  - **开发**：Vite dev server（`5173`）通过 proxy 把 `/api` 转发到后端 `18083`，浏览器只看到单一 origin，无需 CORS。
  - **生产**：`npm run build` 生成 `frontend/dist/`，后端会自动托管其中的 `index.html` 和静态资源。

## 开发

```bash
# 需要 Node 18+（本机用 nvm 装了 v24）
cd frontend
npm install          # 首次
npm run dev          # http://localhost:5173

# 后端需另开终端运行（提供实时数据）
bash backend/run.sh
```

如果后端不在默认地址，可覆盖代理目标：

```bash
VITE_BACKEND_TARGET=http://<ip>:18083 npm run dev
```

## 构建 / 生产

```bash
cd frontend
npm run build        # 产出 frontend/dist/
bash backend/run.sh  # 访问 http://<host>:18083 即可看到构建后的前端
```

## 目录结构

```
src/
├── main.js                     # 入口，挂载 Pinia
├── App.vue                     # 布局 + 轮询生命周期
├── api/client.js               # REST 封装（相对路径）
├── stores/telemetry.js         # Pinia：集中轮询 /api/state（替代旧的 setInterval(tick)）
├── composables/
│   └── useCanvasRenderer.js    # 命令式 canvas 绑定（DPR 缩放 + 状态变化重绘）
├── utils/
│   ├── format.js               # fmt / 角度换算（从旧代码逐字移植）
│   └── scan.js                 # analyzeScan 等激光分析（从旧代码逐字移植）
└── components/
    ├── ConnectionStatus.vue    # 顶部连接状态
    ├── LaserScanPanel.vue      # 样板：命令式 Canvas + 响应式读数/告警
    └── SensorsPanel.vue        # 样板：纯响应式网格
```

## 迁移原则

- **Canvas 保持命令式**：响应式只驱动布局、读数、告警文字；canvas 绘制走 `useCanvasRenderer`，在状态快照更新或元素 resize 时重绘，不把逐帧绘制放进响应式热路径。
- **纯逻辑函数逐字移植**：`utils/scan.js`、`utils/format.js` 直接来自旧代码，便于比对与回归。

## 已迁移 / 待迁移

- [x] 顶部连接状态
- [x] Laser Scan 面板（canvas + 读数 + 告警）
- [x] Ultrasonic / Bumper Sensors 面板
- [ ] SLAM Map + Odometry（地图 canvas、点选、航点、朝向）
- [ ] 动作链 / 工作流
- [ ] 点位库
- [ ] 3D 点云
- [ ] 导航故障快照
- [ ] Mapping Mode
- [ ] Raw State

> 原始整页参考保留在 `frontend/legacy/index.html`（从旧文件抽离而来，仅供迁移比对，不参与构建）。
