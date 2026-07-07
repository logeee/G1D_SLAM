<script setup>
import { computed, ref } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useWorkflowStore } from '../stores/workflow.js'
import { useRelocalizationStore } from '../stores/relocalization.js'
import { useColumnHeightStore } from '../stores/columnHeight.js'
import { fmt } from '../utils/format.js'
import MapCanvas from './MapCanvas.vue'
import WorkflowPanel from './WorkflowPanel.vue'

const telemetry = useTelemetryStore()
const workflow = useWorkflowStore()
const relocalization = useRelocalizationStore()
const columnHeight = useColumnHeightStore()

const { state } = storeToRefs(telemetry)
const { selectedWaypoints, headingMode, navMode, navSpeedRatio, navStatusHtml, navStatusKind, workflowRun, navButtonsBusy } =
  storeToRefs(workflow)
const { statusHtml: relocStatusHtml, statusKind: relocStatusKind, radius: relocRadius, movement: relocMovement, runBusy: relocRunBusy } =
  storeToRefs(relocalization)
const { physicalHeightText, physicalHeightTitle } = storeToRefs(columnHeight)

const headingDegInput = ref('')

const busy = computed(() => navButtonsBusy.value || workflowRun.value.running)

const mapMeta = computed(() => {
  const map = state.value?.map
  return map ? `${map.width}x${map.height}, ${fmt(map.resolution, 3, 'm/cell')}` : 'waiting'
})

const odomX = computed(() => fmt(state.value?.odom?.x, 3, 'm'))
const odomY = computed(() => fmt(state.value?.odom?.y, 3, 'm'))
const odomYaw = computed(() => fmt(state.value?.odom?.yaw_deg, 1, 'deg'))
const trackCount = computed(() => (state.value?.track ? String(state.value.track.length) : '--'))
const waypointCount = computed(() => String(selectedWaypoints.value.length))
const planCount = computed(() => {
  const plan = state.value?.navigation?.global_plan_path
  return plan ? `${plan.total_poses}` : '--'
})
const localizationState = computed(() => {
  const basic = state.value?.navigation?.robot_basic_state
  return basic ? `${basic.is_localization_enabled ? 'ON' : 'OFF'} / ${basic.localization_quality}` : '--'
})
const navCommand = computed(() => {
  const cmd = state.value?.navigation?.last_command
  return cmd?.type ? `${cmd.type} #${cmd.seq || ''}` : '--'
})
const targetYawText = computed(() => {
  const t = workflow.computeTargetYaw(state.value)
  return t ? `${t.yawDeg.toFixed(1)}° ${t.label}` : '--'
})

function applyHeading() {
  const applied = workflow.applyHeadingDegFromInput(headingDegInput.value)
  if (applied !== null && applied !== undefined) headingDegInput.value = applied.toFixed(1)
}

function onDirectChange(ev) {
  const enabled = ev.target.checked
  workflow.navMode = enabled ? 'direct' : 'normal'
  workflow.showNavMessage(
    enabled ? 'warn' : '',
    enabled ? '导航：已启用 <strong>直连不绕障</strong>，Slamware 会按指定路径走，遇障停止。' : '导航：已切回 <strong>普通避障</strong>。',
  )
  telemetry.tick()
}

function onRawChange(ev) {
  const enabled = ev.target.checked
  workflow.navMode = enabled ? 'raw' : 'normal'
  workflow.showNavMessage(
    enabled ? 'bad' : '',
    enabled
      ? '导航：已启用 <strong>裸控无避障</strong>，将直接发布 /cmd_vel，请确认路径完全安全。'
      : '导航：已关闭 <strong>裸控无避障</strong>。',
  )
  telemetry.tick()
}
</script>

<template>
  <section class="map-section">
    <div class="panel-head">
      <h2>SLAM Map + Odometry</h2>
      <span class="meta">{{ mapMeta }}</span>
    </div>
    <div class="map-workflow-grid">
      <div>
        <div class="canvas-wrap"><MapCanvas interactive /></div>
        <div class="toolbar">
          <button :disabled="busy" @click="workflow.undoWaypoint()">撤销点</button>
          <button :disabled="busy" @click="workflow.clearWaypoints()">清空点</button>
          <button :class="{ active: headingMode }" :disabled="busy" @click="workflow.toggleHeadingMode()">设置朝向</button>
          <button :disabled="busy" @click="workflow.clearHeading(); headingDegInput = ''">清除朝向</button>
          <input
            v-model="headingDegInput"
            class="heading-input"
            type="number"
            step="1"
            min="-180"
            max="180"
            placeholder="角度°"
            :disabled="busy"
            @keydown.enter="applyHeading"
          />
          <button :disabled="busy" @click="applyHeading">应用角度</button>
          <label class="speed-ratio-label" title="Slamware speed_ratio，1.0 为最大">
            导航速度
            <input v-model.number="navSpeedRatio" type="number" step="0.05" min="0.05" max="1" :disabled="busy" />
          </label>
          <label class="nav-mode-toggle" title="使用 Slamware KeyPoints 模式：按指定路径走，不自动绕障；遇到障碍会停止。">
            <input type="checkbox" :checked="navMode === 'direct'" :disabled="busy" @change="onDirectChange" />
            直连不绕障
          </label>
          <label class="nav-mode-toggle" title="绕开 Slamware 导航，直接发布 /cmd_vel；不会自动避障，请只在确认路径安全时使用。">
            <input type="checkbox" :checked="navMode === 'raw'" :disabled="busy" @change="onRawChange" />
            裸控无避障
          </label>
          <label class="speed-ratio-label" title="Slamware recover_localization 搜索半径；默认假设关机后机器人没有移动。">
            重定位半径
            <input v-model.number="relocRadius" type="number" step="0.05" min="0.05" max="3" :disabled="busy" />
          </label>
          <label class="speed-ratio-label" title="默认不移动；如果静态匹配不够，再手动选择原地旋转。">
            重定位方式
            <select v-model="relocMovement" :disabled="busy">
              <option value="NO_MOVE">不移动</option>
              <option value="ROTATE_ONLY">原地旋转</option>
              <option value="ANY">允许移动</option>
            </select>
          </label>
          <button :disabled="busy" title="把当前 odom 保存为下次开机重定位基准" @click="relocalization.saveAnchor()">保存开机基准</button>
          <button :disabled="busy || relocRunBusy" title="用保存的基准位姿 set_pose，并在基准周围调用 Slamware recover_localization" @click="relocalization.runRelocalization()">
            按基准重定位
          </button>
          <span class="nav-hint">{{ headingMode ? '在地图上点击终点需要朝向的方向' : '在地图上点击快速增加导航动作' }}</span>
        </div>
        <div class="nav-status" :class="navStatusKind" v-html="navStatusHtml"></div>
        <div class="nav-status" :class="relocStatusKind" v-html="relocStatusHtml"></div>
        <div class="readout">
          <div class="metric"><div class="label">X</div><div class="value">{{ odomX }}</div></div>
          <div class="metric"><div class="label">Y</div><div class="value">{{ odomY }}</div></div>
          <div class="metric"><div class="label">Yaw</div><div class="value">{{ odomYaw }}</div></div>
          <div class="metric"><div class="label">立柱高度</div><div class="value" :title="physicalHeightTitle">{{ physicalHeightText }}</div></div>
          <div class="metric"><div class="label">Track</div><div class="value">{{ trackCount }}</div></div>
          <div class="metric"><div class="label">航点</div><div class="value">{{ waypointCount }}</div></div>
          <div class="metric"><div class="label">规划路径</div><div class="value">{{ planCount }}</div></div>
          <div class="metric"><div class="label">定位</div><div class="value">{{ localizationState }}</div></div>
          <div class="metric"><div class="label">导航指令</div><div class="value">{{ navCommand }}</div></div>
          <div class="metric"><div class="label">目标角度</div><div class="value">{{ targetYawText }}</div></div>
        </div>
      </div>
      <WorkflowPanel />
    </div>
  </section>
</template>
