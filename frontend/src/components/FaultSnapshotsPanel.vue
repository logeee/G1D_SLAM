<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useWorkflowStore } from '../stores/workflow.js'
import { api } from '../api/client.js'
import { fmt } from '../utils/format.js'

const telemetry = useTelemetryStore()
const workflow = useWorkflowStore()
const { state } = storeToRefs(telemetry)

const FAULT_REASON_MAP = {
  navigation_start: '开始导航',
  navigation_start_blocked: '开始前安全检查失败',
  raw_navigation_start: '裸控导航开始',
  raw_navigation_start_blocked: '裸控导航被拒绝',
  navigation_cancel: '停止/取消导航',
  global_plan_zero: '规划路径为 0',
  odom_still_during_navigation: '里程计 3 秒不动',
  movement_after_cancel: '停止后仍有位姿变化',
  sensor_impact: '传感器触发',
  laser_close_during_navigation: '导航中激光近障碍',
  frontend_navigation_stall: '前端判定导航卡住',
  frontend_navigation_timeout: '前端等待导航超时',
  manual_fault_snapshot: '手动记录',
}

function faultReasonText(reason) {
  return FAULT_REASON_MAP[reason] || reason || '--'
}

function faultSeverityClass(snap) {
  const reason = snap?.reason || ''
  const hits = snap?.sensors?.hits || []
  const scanMin = Number(snap?.scan?.min_range)
  if (
    reason.includes('blocked') ||
    reason.includes('impact') ||
    reason.includes('movement_after_cancel') ||
    hits.length ||
    (Number.isFinite(scanMin) && scanMin <= 0.45)
  )
    return 'danger'
  if (
    reason.includes('plan_zero') ||
    reason.includes('odom_still') ||
    reason.includes('laser_close') ||
    (Number.isFinite(scanMin) && scanMin <= 0.8)
  )
    return 'warn'
  return ''
}

const snapshots = computed(() => state.value?.fault_snapshots || [])

const meta = computed(() => {
  const s = snapshots.value
  return s.length ? `${s.length} 条 / 最新 #${s[s.length - 1]?.seq || '--'}` : '0 条'
})

const items = computed(() =>
  snapshots.value
    .slice(-12)
    .reverse()
    .map(snap => {
      const cmd = snap.navigation?.last_command || {}
      const plan = snap.navigation?.global_plan_path || {}
      const odom = snap.odom || {}
      const goal = snap.goal || {}
      const scan = snap.scan || {}
      const hits = snap.sensors?.hits || []
      const close = scan.close_counts || {}
      const errors = snap.recent_slamware_errors?.lines || []
      const goalText = goal.target
        ? `目标 (${fmt(goal.target.x, 3, 'm')}, ${fmt(goal.target.y, 3, 'm')}) / 距离 ${fmt(goal.distance_m, 3, 'm')} / 相对角 ${fmt(goal.bearing_robot_deg, 1, 'deg')}`
        : '目标 --'
      const hitText = hits.length
        ? hits
            .map(h => `${h.sensor_type_name || h.sensor_type}#${h.id}${h.value !== null && h.value !== undefined ? '=' + h.value : ''}`)
            .join('，')
        : '无'
      return {
        cls: faultSeverityClass(snap),
        seq: snap.seq || '--',
        reason: faultReasonText(snap.reason),
        capturedAt: snap.captured_at || '',
        cmdType: cmd.type || '--',
        cmdSeq: cmd.seq || '--',
        planPoses: plan.total_poses ?? '--',
        quality: snap.navigation?.robot_basic_state?.localization_quality ?? '--',
        poseText: `${fmt(odom.x, 3, 'm')}, ${fmt(odom.y, 3, 'm')}, ${fmt(odom.yaw_deg, 1, 'deg')}`,
        scanMin: fmt(scan.min_range, 3, 'm'),
        hitText,
        goalText,
        closeText: `0.6m内 ${close.lt_0_6m ?? '--'}，0.8m内 ${close.lt_0_8m ?? '--'}`,
        errorText: errors.length ? errors.slice(-4).join('\n') : '无最近 Slamware 错误',
        key: snap.seq ?? snap.captured_at ?? Math.random(),
      }
    }),
)

async function recordManual() {
  let data
  try {
    data = await api.post('/api/fault_snapshots/log', {
      reason: 'manual_fault_snapshot',
      source: 'frontend',
      note: 'button clicked on dashboard',
      workflow: workflow.workflowRun,
    })
  } catch (err) {
    data = { ok: false, error: String(err) }
  }
  workflow.showNavMessage(
    data.ok ? 'warn' : 'bad',
    data.ok ? `快照：<strong>已记录</strong> #${data.snapshot?.seq || '--'}` : `快照：<strong>记录失败</strong> ${data.error || ''}`,
  )
  telemetry.tick()
}

async function clearAll() {
  try {
    const data = await api.post('/api/fault_snapshots/clear', {})
    workflow.showNavMessage(data.ok ? '' : 'bad', data.ok ? '快照：已清空页面缓存，落盘 jsonl 不删除。' : `快照：清空失败 ${data.error || ''}`)
    telemetry.tick()
  } catch (err) {
    workflow.showNavMessage('bad', `快照：清空失败 ${err}`)
  }
}
</script>

<template>
  <section>
    <div class="panel-head">
      <h2>导航故障快照</h2>
      <span class="meta">{{ meta }}</span>
    </div>
    <div class="fault-toolbar">
      <button @click="recordManual">记录当前状态</button>
      <button class="danger" @click="clearAll">清空页面快照</button>
      <span class="meta">自动记录：开始导航 / plan=0 / 里程计不动 / 停止 / 传感器触发</span>
    </div>
    <div class="fault-list">
      <div v-if="!items.length" class="fault-empty">暂无快照。导航开始、plan=0、停止、传感器触发或手动记录时会出现在这里。</div>
      <div v-for="item in items" :key="item.key" class="fault-item" :class="item.cls">
        <div class="fault-title"><span>#{{ item.seq }} {{ item.reason }}</span><small>{{ item.capturedAt }}</small></div>
        <div class="fault-grid">
          <div>命令 <strong>{{ item.cmdType }} #{{ item.cmdSeq }}</strong></div>
          <div>规划 <strong>{{ item.planPoses }} 点</strong></div>
          <div>定位 <strong>{{ item.quality }}</strong></div>
          <div>位姿 <strong>{{ item.poseText }}</strong></div>
          <div>激光最近 <strong>{{ item.scanMin }}</strong></div>
          <div>传感器 <strong>{{ item.hitText }}</strong></div>
          <div style="grid-column: 1 / -1;">{{ item.goalText }}</div>
          <div style="grid-column: 1 / -1;">近障碍统计：<strong>{{ item.closeText }}</strong></div>
        </div>
        <pre class="fault-errors">{{ item.errorText }}</pre>
      </div>
    </div>
  </section>
</template>
