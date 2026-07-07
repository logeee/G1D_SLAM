import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'
import { normalizeAngle, radToDeg, clampSpeedRatio, angleDiffDeg } from '../utils/format.js'
import { useTelemetryStore } from './telemetry.js'
import { usePointsStore } from './points.js'

const WORKFLOW_CACHE_KEY = 'g1d_slam_workflow_actions_v2'
const NAV_REACH_DISTANCE_M = 0.18
const NAV_REACH_YAW_DEG = 4.0
const NAV_REACH_STABLE_MS = 1200
const NAV_IDLE_ACCEPT_DISTANCE_M = 0.22
const NAV_IDLE_ACCEPT_YAW_DEG = 12.0
const NAV_IDLE_STABLE_MS = 1800
const NAV_ODOM_STILL_M = 0.008
const NAV_ODOM_STILL_YAW_DEG = 0.8

function makeActionId(prefix = 'act') {
  return `${prefix}-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`
}

function sleepMs(ms) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

function armActionTitle(action) {
  const phase = String(action.phase || '').toUpperCase()
  const target = action.targetObject || action.target_object || ''
  if (phase === 'PICK') return `机械臂抓取 ${target || '目标'}`
  if (phase === 'PLACE') return '机械臂放置'
  if (phase === 'RESET') return '机械臂复位'
  return '机械臂任务'
}

function newRun(note = '待执行', extra = {}) {
  return {
    running: false,
    mode: 'idle',
    currentIndex: -1,
    completed: {},
    error: '',
    navigationStartedAt: 0,
    actionStartedAt: 0,
    actionDurationSec: 5,
    startPose: null,
    note,
    ...extra,
  }
}

export const useWorkflowStore = defineStore('workflow', () => {
  const telemetry = useTelemetryStore()
  const points = usePointsStore()

  const workflowActions = ref([])
  const selectedWaypoints = ref([])
  const headingMode = ref(false)
  const finalHeadingPoint = ref(null)
  const manualHeadingDeg = ref(null)
  const editingActionId = ref(null)
  const draggedActionId = ref(null)
  const workflowRun = ref(newRun())
  const navButtonsBusy = ref(false)

  // Navigation mode + speed (legacy checkbox/inputs).
  const navMode = ref('normal') // normal | direct | raw
  const navSpeedRatio = ref(1)

  // Shared #navStatus line: computed summary each tick, transiently overwritten
  // by showNavMessage (mirrors the legacy behavior where tick() overwrites it).
  const navStatusHtml = ref('导航：等待选点')
  const navStatusKind = ref('')

  function lastState() {
    return telemetry.state
  }

  // ---- cache ----
  function saveWorkflowCache() {
    try {
      window.localStorage.setItem(
        WORKFLOW_CACHE_KEY,
        JSON.stringify({ version: 2, savedAt: Date.now(), actions: workflowActions.value }),
      )
    } catch (err) {
      console.warn('failed to save workflow cache', err)
    }
  }

  function loadWorkflowCache() {
    try {
      const raw = window.localStorage.getItem(WORKFLOW_CACHE_KEY)
      if (!raw) return
      const parsed = JSON.parse(raw)
      const actions = Array.isArray(parsed) ? parsed : parsed.actions
      if (!Array.isArray(actions)) return
      workflowActions.value = actions
        .map(a => (a && typeof a === 'object' ? { ...a, id: a.id || makeActionId(a.type || 'act') } : null))
        .filter(Boolean)
      syncSelectedWaypointsFromActions()
    } catch (err) {
      console.warn('failed to load workflow cache', err)
    }
  }

  function clearWorkflowEditMode() {
    editingActionId.value = null
  }

  // ---- resolve against saved points ----
  function resolveWorkflowAction(action) {
    if (!action || action.type !== 'navigate' || !action.pointId) return action
    const point = points.getById(action.pointId)
    if (!point) return { ...action, pointMissing: true }
    return {
      ...action,
      title: `导航到 ${point.name || '点位'}`,
      pointName: point.name || '',
      x: Number(Number(point.x).toFixed(4)),
      y: Number(Number(point.y).toFixed(4)),
      yawDeg: Number(radToDeg(normalizeAngle((Number(point.yaw_deg || 0) * Math.PI) / 180)).toFixed(3)),
      pointMissing: false,
    }
  }

  function getResolvedWorkflowActions() {
    return workflowActions.value.map(resolveWorkflowAction)
  }

  function refreshLinkedWorkflowActions() {
    let changed = false
    workflowActions.value = workflowActions.value.map(action => {
      const resolved = resolveWorkflowAction(action)
      if (!resolved || resolved === action || resolved.pointMissing) return action
      const next = {
        ...action,
        title: resolved.title,
        pointName: resolved.pointName,
        x: resolved.x,
        y: resolved.y,
        yawDeg: resolved.yawDeg,
      }
      if (JSON.stringify(next) !== JSON.stringify(action)) changed = true
      return next
    })
    if (changed) saveWorkflowCache()
    syncSelectedWaypointsFromActions()
  }

  function getNavigationActions() {
    return getResolvedWorkflowActions().filter(action => action.type === 'navigate')
  }

  function getLastNavigationAction() {
    for (let i = workflowActions.value.length - 1; i >= 0; i--) {
      if (workflowActions.value[i].type === 'navigate') return workflowActions.value[i]
    }
    return null
  }

  function syncSelectedWaypointsFromActions() {
    const navActions = getNavigationActions()
    selectedWaypoints.value = navActions.map(action => ({
      x: Number(action.x),
      y: Number(action.y),
      actionId: action.id,
    }))
    const lastNav = navActions[navActions.length - 1]
    manualHeadingDeg.value = lastNav && Number.isFinite(Number(lastNav.yawDeg)) ? Number(lastNav.yawDeg) : null
  }

  function resetWorkflowAfterEdit() {
    syncSelectedWaypointsFromActions()
    resetWorkflowRun('待执行')
  }

  function setLastNavigationYaw(yawDeg) {
    const action = getLastNavigationAction()
    if (!action) return false
    action.yawDeg = Number(radToDeg(normalizeAngle((Number(yawDeg) * Math.PI) / 180)).toFixed(3))
    saveWorkflowCache()
    syncSelectedWaypointsFromActions()
    return true
  }

  function addWorkflowAction(action) {
    workflowActions.value = [...workflowActions.value, action]
    saveWorkflowCache()
    resetWorkflowAfterEdit()
  }

  function removeWorkflowAction(actionId) {
    workflowActions.value = workflowActions.value.filter(action => action.id !== actionId)
    if (editingActionId.value === actionId) clearWorkflowEditMode()
    saveWorkflowCache()
    resetWorkflowAfterEdit()
  }

  function reorderWorkflowAction(dragId, targetId) {
    if (!dragId || !targetId || dragId === targetId) return
    const list = [...workflowActions.value]
    const from = list.findIndex(action => action.id === dragId)
    const to = list.findIndex(action => action.id === targetId)
    if (from < 0 || to < 0) return
    const [item] = list.splice(from, 1)
    list.splice(to, 0, item)
    workflowActions.value = list
    saveWorkflowCache()
    resetWorkflowAfterEdit()
  }

  function commitWorkflowAction(action) {
    if (!editingActionId.value) {
      addWorkflowAction(action)
      return 'added'
    }
    const list = [...workflowActions.value]
    const idx = list.findIndex(item => item.id === editingActionId.value)
    action.id = editingActionId.value
    if (idx >= 0) list[idx] = action
    else list.push(action)
    workflowActions.value = list
    clearWorkflowEditMode()
    saveWorkflowCache()
    resetWorkflowAfterEdit()
    return 'edited'
  }

  // ---- target yaw ----
  function computeTargetYaw(state = lastState()) {
    if (!selectedWaypoints.value.length) return null
    const lastPoint = selectedWaypoints.value[selectedWaypoints.value.length - 1]
    if (manualHeadingDeg.value !== null && manualHeadingDeg.value !== undefined) {
      const yaw = normalizeAngle((Number(manualHeadingDeg.value) * Math.PI) / 180)
      return { yaw, yawDeg: radToDeg(yaw), source: 'manual_heading_input_deg', label: '输入' }
    }
    if (finalHeadingPoint.value) {
      const dx = finalHeadingPoint.value.x - lastPoint.x
      const dy = finalHeadingPoint.value.y - lastPoint.y
      if (Math.hypot(dx, dy) > 0.001) {
        const yaw = normalizeAngle(Math.atan2(dy, dx))
        return { yaw, yawDeg: radToDeg(yaw), source: 'manual_heading_arrow', label: '手动' }
      }
    }
    if (selectedWaypoints.value.length >= 2) {
      const prev = selectedWaypoints.value[selectedWaypoints.value.length - 2]
      const dx = lastPoint.x - prev.x
      const dy = lastPoint.y - prev.y
      if (Math.hypot(dx, dy) > 0.001) {
        const yaw = normalizeAngle(Math.atan2(dy, dx))
        return { yaw, yawDeg: radToDeg(yaw), source: 'auto_last_segment', label: '自动' }
      }
    }
    if (state?.odom?.yaw !== null && state?.odom?.yaw !== undefined) {
      const yaw = normalizeAngle(Number(state.odom.yaw))
      return { yaw, yawDeg: radToDeg(yaw), source: 'current_odom', label: '当前' }
    }
    return null
  }

  // ---- reach / progress helpers ----
  function distanceToAction(action, state = lastState()) {
    if (!state?.odom || action?.x === undefined || action?.y === undefined) return null
    return Math.hypot(Number(state.odom.x) - Number(action.x), Number(state.odom.y) - Number(action.y))
  }

  function navigationReachState(action, state = lastState()) {
    const dist = distanceToAction(action, state)
    const distanceOk = dist !== null && dist <= NAV_REACH_DISTANCE_M
    const hasTargetYaw = Number.isFinite(Number(action?.yawDeg))
    const yawErrorDeg = hasTargetYaw ? angleDiffDeg(state?.odom?.yaw_deg, action.yawDeg) : null
    const yawOk = hasTargetYaw ? yawErrorDeg !== null && yawErrorDeg <= NAV_REACH_YAW_DEG : true
    return { reached: Boolean(distanceOk && yawOk), distanceOk, yawOk, dist, yawErrorDeg }
  }

  function isActionReached(action, state = lastState()) {
    return navigationReachState(action, state).reached
  }

  function previousNavigationPose(actionIndex) {
    for (let i = actionIndex - 1; i >= 0; i--) {
      const action = resolveWorkflowAction(workflowActions.value[i])
      if (action?.type === 'navigate') return action
    }
    return workflowRun.value.startPose
  }

  function estimateNavProgress(action, state = lastState()) {
    const dist = distanceToAction(action, state)
    if (dist === null) return 0
    const start = previousNavigationPose(workflowActions.value.findIndex(item => item.id === action.id))
    const total = start
      ? Math.hypot(Number(action.x) - Number(start.x), Number(action.y) - Number(start.y))
      : Math.max(dist, 0.01)
    if (total < 0.01) return isActionReached(action, state) ? 1 : 0
    return Math.max(0, Math.min(1, 1 - dist / total))
  }

  function getWorkflowModules() {
    const resolvedActions = getResolvedWorkflowActions()
    const resolvedNavActions = resolvedActions.filter(action => action.type === 'navigate')
    return resolvedActions.map((action, index) => {
      if (action.type === 'navigate') {
        return {
          ...action,
          title: action.title || '导航',
          speedRatio: clampSpeedRatio(action.speedRatio, 1),
          index,
          navIndex: resolvedNavActions.findIndex(nav => nav.id === action.id),
        }
      }
      if (action.type === 'arm_task') {
        return { ...action, title: action.title || armActionTitle(action), timeoutSec: action.timeoutSec || 120, index }
      }
      if (action.type === 'column_height') {
        return {
          ...action,
          title: action.title || '立柱升降',
          targetPhysicalHeightM: action.targetPhysicalHeightM ?? action.target_physical_height_m,
          targetHeightM: action.targetHeightM ?? action.target_height_m,
          timeoutSec: action.timeoutSec || 30,
          index,
        }
      }
      return { ...action, title: action.title || '拾取熊猫烟', durationSec: action.durationSec || 5, index }
    })
  }

  function actionTitle(action, index) {
    if (action.type === 'navigate') return `${index + 1}. ${action.title || '导航'}`
    if (action.type === 'arm_task') return `${index + 1}. ${action.title || armActionTitle(action)}`
    if (action.type === 'column_height') return `${index + 1}. ${action.title || '立柱升降'}`
    if (action.type === 'fake_pick_xiongmao') return `${index + 1}. 拾取熊猫烟`
    return `${index + 1}. ${action.title || action.type || '动作'}`
  }

  function actionDetail(action) {
    if (action.type === 'navigate') {
      const yawText = Number.isFinite(Number(action.yawDeg)) ? `, yaw=${Number(action.yawDeg).toFixed(1)}°` : ''
      const pointText = action.pointName ? `，点位库：${action.pointName}` : ''
      const speedText = `，速度 ${clampSpeedRatio(action.speedRatio, 1).toFixed(2)}`
      const missingText = action.pointMissing ? '，点位库未找到，使用卡片缓存坐标' : ''
      return `目标 x=${Number(action.x).toFixed(3)}m, y=${Number(action.y).toFixed(3)}m${yawText}${pointText}${speedText}${missingText}`
    }
    if (action.type === 'fake_pick_xiongmao') {
      return `假动作模块：后端休眠 ${action.durationSec || 5}s，后续可替换为机械臂动作`
    }
    if (action.type === 'arm_task') {
      const phase = String(action.phase || '').toUpperCase()
      const target = action.targetObject || action.target_object || ''
      const targetText = target ? `，目标=${target}` : ''
      return `ROS 手臂任务：phase=${phase}${targetText}，超时 ${action.timeoutSec || 120}s`
    }
    if (action.type === 'column_height') {
      const physical = action.targetPhysicalHeightM ?? action.target_physical_height_m
      if (physical !== undefined && physical !== null) {
        return `G1D 立柱物理高度：target=${Number(physical || 0).toFixed(3)}m，后端自动换算 raw，超时 ${action.timeoutSec || 30}s`
      }
      return `G1D 立柱 raw 高度：target=${Number(action.targetHeightM || 0).toFixed(3)}m（旧动作），超时 ${action.timeoutSec || 30}s`
    }
    return '预留动作模块'
  }

  function workflowStepStatus(module, index) {
    const run = workflowRun.value
    if (run.error && index === run.currentIndex) return 'error'
    if (run.completed[module.id]) return 'done'
    if (run.running && index === run.currentIndex) return 'running'
    return 'queued'
  }

  function workflowStepProgress(module, index) {
    const status = workflowStepStatus(module, index)
    if (status === 'done') return 100
    if (status !== 'running') return 0
    const run = workflowRun.value
    if (module.type === 'navigate') return Math.round(estimateNavProgress(module) * 100)
    if (module.type === 'arm_task' || module.type === 'column_height' || module.type === 'fake_pick_xiongmao') {
      if (!run.actionStartedAt) return 0
      const elapsed = (Date.now() - run.actionStartedAt) / 1000
      const total = Math.max(0.1, run.actionDurationSec || module.timeoutSec || module.durationSec || 5)
      return Math.round(Math.max(0, Math.min(1, elapsed / total)) * 100)
    }
    return 0
  }

  // ---- run lifecycle ----
  function resetWorkflowRun(note = '待执行') {
    workflowRun.value = newRun(note)
  }

  function beginWorkflowRun(mode, currentIndex = 0) {
    const odom = lastState()?.odom
    workflowRun.value = newRun(mode === 'chain' ? '动作链执行中' : '导航执行中', {
      running: true,
      mode,
      currentIndex,
      navigationStartedAt: Date.now(),
      startPose: odom ? { x: odom.x, y: odom.y } : null,
    })
  }

  function markWorkflowError(message) {
    const run = { ...workflowRun.value }
    run.running = false
    run.error = message || '执行失败'
    run.note = run.error
    workflowRun.value = run
  }

  function updateWorkflowProgress(state) {
    const run = workflowRun.value
    if (!run.running) return
    const modules = getWorkflowModules()
    if (!modules.length) {
      run.running = false
      run.note = '没有动作模块'
      workflowRun.value = { ...run }
      return
    }
    if (run.mode === 'navigation') {
      modules.forEach(action => {
        if (action.type === 'navigate' && isActionReached(action, state)) run.completed[action.id] = true
      })
      const nextIndex = modules.findIndex(action => action.type === 'navigate' && !run.completed[action.id])
      if (nextIndex >= 0) {
        run.currentIndex = nextIndex
      } else {
        run.running = false
        let lastNavIndex = -1
        modules.forEach((action, index) => {
          if (action.type === 'navigate') lastNavIndex = index
        })
        run.currentIndex = Math.max(0, lastNavIndex)
        run.note = '导航完成'
      }
      workflowRun.value = { ...run }
    }
  }

  function clearWorkflowActions() {
    if (workflowRun.value.running) {
      showNavMessage('bad', '动作链执行中：<strong>请先停止后再清空动作</strong>')
      return
    }
    workflowActions.value = []
    selectedWaypoints.value = []
    finalHeadingPoint.value = null
    manualHeadingDeg.value = null
    draggedActionId.value = null
    clearWorkflowEditMode()
    saveWorkflowCache()
    setHeadingMode(false)
    resetWorkflowRun('已清空动作')
    showNavMessage('', '动作链：<strong>已清空所有动作</strong>')
  }

  // ---- nav status / messages ----
  function showNavMessage(kind, text) {
    navStatusKind.value = kind || ''
    navStatusHtml.value = text
  }

  function updateNavigationStatus(state) {
    const last = state?.navigation?.last_command
    const basic = state?.navigation?.robot_basic_state
    const slamState = state?.navigation?.slamware_state?.state || '--'
    const planCount = state?.navigation?.global_plan_path?.total_poses || 0
    const targetYaw = computeTargetYaw(state)
    const plannedMode = navMode.value === 'raw' ? '裸控无避障' : navMode.value === 'direct' ? '直连不绕障' : '普通避障'
    const lastMode =
      last?.navigation_mode === 'raw_cmd_vel_no_obstacle_avoidance'
        ? '裸控无避障'
        : last?.navigation_mode === 'direct_key_points_stop_on_obstacle'
          ? '直连不绕障'
          : last?.navigation_mode === 'normal_slamware'
            ? '普通避障'
            : '--'
    const parts = [
      `<strong>${selectedWaypoints.value.length}</strong> 个航点`,
      `目标角度: <strong>${targetYaw ? targetYaw.yawDeg.toFixed(1) + '° ' + targetYaw.label : '--'}</strong>`,
      `模式: <strong>${plannedMode}</strong>`,
      `上次模式: <strong>${lastMode}</strong>`,
      `Slamware: <strong>${slamState}</strong>`,
      `定位: <strong>${basic ? (basic.is_localization_enabled ? 'ON' : 'OFF') + ' / ' + basic.localization_quality : '--'}</strong>`,
      `规划路径: <strong>${planCount}</strong> 点`,
    ]
    if (last?.type) parts.push(`上次指令: <strong>${last.type}</strong>`)
    let kind = ''
    if (basic && !basic.is_localization_enabled) kind = 'bad'
    else if (basic && Number(basic.localization_quality) <= 0) kind = 'warn'
    navStatusKind.value = kind
    navStatusHtml.value = `导航：${parts.join('　')}`
  }

  // ---- fault snapshot logging (used by run engine) ----
  async function logFaultSnapshot(reason, extra = {}) {
    try {
      return await api.post('/api/fault_snapshots/log', {
        reason,
        source: 'frontend',
        workflow: workflowRun.value,
        ...extra,
      })
    } catch (err) {
      console.warn('fault snapshot failed', err)
      return { ok: false, error: String(err) }
    }
  }

  // ---- navigation payload + start ----
  function buildNavigationPayload(waypoints = selectedWaypoints.value, yawDeg = null, yawSource = 'workflow_action', speedRatio = null) {
    const payload = { waypoints }
    const resolvedSpeedRatio = clampSpeedRatio(
      speedRatio !== null && speedRatio !== undefined ? speedRatio : navSpeedRatio.value,
      1,
    )
    payload.speed_ratio = Number(resolvedSpeedRatio.toFixed(3))
    if (navMode.value === 'raw') {
      payload.raw_cmd_vel = true
      payload.disable_obstacle_avoidance = true
      payload.navigation_mode = 'raw_cmd_vel_no_obstacle_avoidance'
      payload.raw_linear_speed_mps = 0.35
      payload.raw_angular_speed_radps = 1.2
    } else if (navMode.value === 'direct') {
      payload.direct_no_avoidance = true
      payload.navigation_mode = 'direct_key_points_stop_on_obstacle'
    }
    if (yawDeg !== null && yawDeg !== undefined && Number.isFinite(Number(yawDeg))) {
      payload.yaw = Number(normalizeAngle((Number(yawDeg) * Math.PI) / 180).toFixed(6))
      payload.yaw_source = yawSource
    } else {
      const targetYaw = computeTargetYaw(lastState())
      if (targetYaw) {
        payload.yaw = Number(targetYaw.yaw.toFixed(6))
        payload.yaw_source = targetYaw.source
      }
    }
    return payload
  }

  async function startNavigation(options = {}) {
    const waypoints = options.waypoints || selectedWaypoints.value
    if (!waypoints.length) {
      showNavMessage('bad', '导航：<strong>请先在地图上点击选择至少一个航点</strong>')
      return { ok: false, error: 'no waypoints' }
    }
    const targetYaw =
      options.yawDeg !== undefined && options.yawDeg !== null
        ? { yawDeg: Number(options.yawDeg), label: '动作' }
        : computeTargetYaw(lastState())
    if (!options.fromWorkflow) navButtonsBusy.value = true
    const modeText = navMode.value === 'raw' ? '裸控无避障（/cmd_vel）' : navMode.value === 'direct' ? '直连不绕障（遇障停止）' : '普通避障'
    showNavMessage('', `导航：正在发送航点和目标角度... ${targetYaw ? targetYaw.yawDeg.toFixed(1) + '° ' + targetYaw.label : ''}，模式 ${modeText}`)
    try {
      const data = await api.post(
        '/api/navigation/start',
        buildNavigationPayload(waypoints, options.yawDeg, options.yawSource || 'workflow_action', options.speedRatio),
      )
      if (data.ok) {
        if (!options.fromWorkflow) {
          const firstNavIndex = getWorkflowModules().findIndex(action => action.type === 'navigate')
          beginWorkflowRun(options.workflowMode || 'navigation', Math.max(0, firstNavIndex))
        }
        const warnings = data.command?.safety?.warnings || []
        const warningText = warnings.length ? `，警告：${warnings.join('；')}` : ''
        const yawText =
          data.command?.yaw_deg !== null && data.command?.yaw_deg !== undefined
            ? `，目标角度 ${Number(data.command.yaw_deg).toFixed(1)}°`
            : ''
        const commandModeText = data.command?.raw_cmd_vel
          ? '，裸控无避障（/cmd_vel）'
          : data.command?.direct_no_avoidance
            ? '，直连不绕障（遇障停止）'
            : '，普通避障'
        showNavMessage(warnings.length ? 'warn' : '', `导航：<strong>已开始</strong>，航点 ${waypoints.length} 个${yawText}${commandModeText}${warningText}`)
      } else {
        const blockers = data.safety?.blockers || []
        const details = blockers.length ? `：${blockers.join('；')}` : data.error || 'unknown error'
        showNavMessage('bad', `导航：<strong>启动失败</strong>${details}`)
        if (!options.fromWorkflow) markWorkflowError(details)
      }
      return data
    } catch (err) {
      showNavMessage('bad', `导航：<strong>请求失败</strong> ${err}`)
      if (!options.fromWorkflow) markWorkflowError(String(err))
      return { ok: false, error: String(err) }
    } finally {
      if (!options.fromWorkflow) navButtonsBusy.value = false
      telemetry.tick()
    }
  }

  // ---- run waiters ----
  async function waitForActionReached(action, timeoutMs = 180000) {
    const started = Date.now()
    let stableSince = null
    let lastReach = null
    while (workflowRun.value.running && Date.now() - started < timeoutMs) {
      lastReach = navigationReachState(action, lastState())
      if (lastReach.reached) {
        if (stableSince === null) stableSince = Date.now()
        if (Date.now() - stableSince >= NAV_REACH_STABLE_MS) return true
      } else {
        stableSince = null
      }
      await sleepMs(250)
    }
    if (!workflowRun.value.running) throw new Error('动作链已停止')
    const distText = lastReach?.dist == null ? '--' : `${(lastReach.dist * 1000).toFixed(0)}mm`
    const yawText = lastReach?.yawErrorDeg == null ? '--' : `${lastReach.yawErrorDeg.toFixed(1)}°`
    throw new Error(`等待导航到达超时：距离偏差 ${distText}，yaw 偏差 ${yawText}`)
  }

  async function waitForSlamwareNavigationComplete(action, timeoutMs = 180000) {
    const started = Date.now()
    let strictStableSince = null
    let planIdleSince = null
    let odomStillSince = null
    let lastReach = null
    let lastOdom = null
    let lastStateError = null
    let sawNavigationEvidence = false
    let loggedPlanStall = false
    while (workflowRun.value.running && Date.now() - started < timeoutMs) {
      let state
      try {
        state = await telemetry.fetchOnce()
      } catch (err) {
        lastStateError = err
        await sleepMs(300)
        continue
      }
      const now = Date.now()
      lastReach = navigationReachState(action, state)
      if (lastReach.reached) {
        if (strictStableSince === null) strictStableSince = now
        if (now - strictStableSince >= NAV_REACH_STABLE_MS) return true
      } else {
        strictStableSince = null
      }
      const plan = state?.navigation?.global_plan_path
      const planCount = Number(plan?.total_poses || 0)
      const planFresh = Number(state?.freshness_s?.global_plan_path)
      const planFreshOk = !Number.isFinite(planFresh) || planFresh <= 2.0
      if (planCount > 0) sawNavigationEvidence = true
      if (plan && planFreshOk && planCount === 0) {
        if (planIdleSince === null) planIdleSince = now
      } else {
        planIdleSince = null
      }
      const odom = state?.odom
      if (odom && lastOdom) {
        const moved = Math.hypot(Number(odom.x) - Number(lastOdom.x), Number(odom.y) - Number(lastOdom.y))
        const yawMoved = angleDiffDeg(odom.yaw_deg, lastOdom.yaw_deg)
        if (moved > 0.015 || (yawMoved !== null && yawMoved > 1.5)) sawNavigationEvidence = true
        if (moved <= NAV_ODOM_STILL_M && (yawMoved === null || yawMoved <= NAV_ODOM_STILL_YAW_DEG)) {
          if (odomStillSince === null) odomStillSince = now
        } else {
          odomStillSince = null
        }
      }
      if (odom) lastOdom = { x: Number(odom.x), y: Number(odom.y), yaw_deg: Number(odom.yaw_deg) }

      const hasTargetYaw = Number.isFinite(Number(action?.yawDeg))
      const idleDistanceOk = lastReach?.dist != null && lastReach.dist <= NAV_IDLE_ACCEPT_DISTANCE_M
      const idleYawOk = hasTargetYaw ? lastReach?.yawErrorDeg != null && lastReach.yawErrorDeg <= NAV_IDLE_ACCEPT_YAW_DEG : true
      const planIdleOk = planIdleSince !== null && now - planIdleSince >= NAV_IDLE_STABLE_MS
      const odomStillOk = odomStillSince !== null && now - odomStillSince >= NAV_IDLE_STABLE_MS
      if (!loggedPlanStall && planIdleOk && odomStillOk && !lastReach?.reached) {
        loggedPlanStall = true
        await logFaultSnapshot('frontend_navigation_stall', {
          action,
          lastReach,
          planCount,
          planIdleMs: planIdleSince === null ? null : now - planIdleSince,
          odomStillMs: odomStillSince === null ? null : now - odomStillSince,
        })
      }
      if (planIdleOk && odomStillOk && idleDistanceOk && idleYawOk && (sawNavigationEvidence || now - started > 3000)) {
        if (!lastReach.reached) {
          const distText = `${(lastReach.dist * 1000).toFixed(0)}mm`
          const yawText = lastReach.yawErrorDeg == null ? '--' : `${lastReach.yawErrorDeg.toFixed(1)}°`
          showNavMessage('warn', `动作链：底层导航已停止，按兜底条件继续；距离 ${distText}，yaw ${yawText}`)
        }
        return true
      }
      await sleepMs(250)
    }
    if (!workflowRun.value.running) throw new Error('动作链已停止')
    if (lastStateError && !lastReach) throw new Error(`读取底盘状态失败：${lastStateError.message || lastStateError}`)
    const distText = lastReach?.dist == null ? '--' : `${(lastReach.dist * 1000).toFixed(0)}mm`
    const yawText = lastReach?.yawErrorDeg == null ? '--' : `${lastReach.yawErrorDeg.toFixed(1)}°`
    await logFaultSnapshot('frontend_navigation_timeout', { action, lastReach, timeoutMs })
    throw new Error(`等待导航到达超时：距离偏差 ${distText}，yaw 偏差 ${yawText}`)
  }

  async function waitForRawNavigationComplete(command, action, timeoutMs = 180000) {
    const rawNavId = command?.raw_nav_id
    if (!rawNavId) return waitForActionReached(action, timeoutMs)
    const started = Date.now()
    let lastRawCommand = null
    let lastStateError = null
    while (workflowRun.value.running && Date.now() - started < timeoutMs) {
      let state
      try {
        state = await telemetry.fetchOnce()
      } catch (err) {
        lastStateError = err
        await sleepMs(300)
        continue
      }
      const current = state?.navigation?.last_command
      if (current?.raw_nav_id === rawNavId) {
        lastRawCommand = current
        const status = String(current.raw_nav_status || '')
        if (status === 'done' || status === 'dry_run') return true
        if (status === 'timeout' || status === 'cancelled' || status === 'error') {
          throw new Error(`裸控导航${status}：${current.raw_nav_error || 'no detail'}`)
        }
      } else if (current?.raw_cmd_vel && current?.raw_nav_id && current.raw_nav_id !== rawNavId) {
        throw new Error('裸控导航被新的 /cmd_vel 导航请求替换')
      }
      await sleepMs(250)
    }
    if (!workflowRun.value.running) throw new Error('动作链已停止')
    if (lastStateError && !lastRawCommand) throw new Error(`读取底盘状态失败：${lastStateError.message || lastStateError}`)
    const statusText = lastRawCommand?.raw_nav_status || '--'
    const distText = lastRawCommand?.raw_nav_distance_m == null ? '--' : `${(Number(lastRawCommand.raw_nav_distance_m) * 1000).toFixed(0)}mm`
    const yawText = lastRawCommand?.raw_nav_yaw_error_deg == null ? '--' : `${Number(lastRawCommand.raw_nav_yaw_error_deg).toFixed(1)}°`
    throw new Error(`等待裸控导航完成超时：status=${statusText}，距离 ${distText}，yaw ${yawText}`)
  }

  async function runWorkflow() {
    const modules = getWorkflowModules()
    if (!modules.length) {
      showNavMessage('bad', '动作链：<strong>请先增加至少一个动作</strong>')
      return
    }
    beginWorkflowRun('chain', 0)
    try {
      for (let index = 0; index < modules.length; index++) {
        const action = modules[index]
        const run = { ...workflowRun.value }
        run.currentIndex = index
        run.actionStartedAt = 0
        workflowRun.value = run
        if (action.type === 'navigate') {
          showNavMessage('', `动作链：正在执行第 ${index + 1} 步导航...`)
          const navData = await startNavigation({
            fromWorkflow: true,
            workflowMode: 'chain',
            waypoints: [{ x: Number(action.x), y: Number(action.y) }],
            yawDeg: action.yawDeg,
            yawSource: 'workflow_action',
            speedRatio: action.speedRatio,
          })
          if (!navData.ok) throw new Error(navData.error || '导航启动失败')
          if (navData.command?.raw_cmd_vel && navData.command?.raw_nav_id) {
            await waitForRawNavigationComplete(navData.command, action)
          } else {
            await waitForSlamwareNavigationComplete(action)
          }
          workflowRun.value.completed[action.id] = true
          continue
        }
        if (action.type === 'arm_task') {
          workflowRun.value.actionStartedAt = Date.now()
          workflowRun.value.actionDurationSec = action.timeoutSec || 120
          workflowRun.value = { ...workflowRun.value }
          showNavMessage('', `动作链：正在执行第 ${index + 1} 步“${action.title || armActionTitle(action)}”...`)
          const actionData = await api.post('/api/actions/execute', {
            type: 'arm_task',
            phase: action.phase,
            target_object: action.targetObject || '',
            timeout_sec: workflowRun.value.actionDurationSec,
            name: action.title || armActionTitle(action),
          })
          if (!actionData.ok) {
            const statusText = actionData.final_status?.status_text || actionData.last_status?.status_text || ''
            throw new Error(actionData.error || statusText || '机械臂任务执行失败')
          }
          workflowRun.value.completed[action.id] = true
          continue
        }
        if (action.type === 'column_height') {
          workflowRun.value.actionStartedAt = Date.now()
          workflowRun.value.actionDurationSec = action.timeoutSec || 30
          workflowRun.value = { ...workflowRun.value }
          const physicalTarget = action.targetPhysicalHeightM ?? action.target_physical_height_m
          const rawTarget = action.targetHeightM ?? action.target_height_m
          const targetText =
            physicalTarget !== undefined && physicalTarget !== null
              ? `物理高度=${Number(physicalTarget || 0).toFixed(3)}m`
              : `raw target=${Number(rawTarget || 0).toFixed(3)}m`
          showNavMessage('', `动作链：正在执行第 ${index + 1} 步“立柱升降” ${targetText}...`)
          const payload = { type: 'column_height', timeout_sec: workflowRun.value.actionDurationSec, name: action.title || '立柱升降' }
          if (physicalTarget !== undefined && physicalTarget !== null) payload.target_physical_height_m = Number(physicalTarget || 0)
          else payload.target_height_m = Number(rawTarget || 0)
          const actionData = await api.post('/api/actions/execute', payload)
          if (!actionData.ok) throw new Error(actionData.error || '立柱升降执行失败')
          workflowRun.value.completed[action.id] = true
          continue
        }
        if (action.type === 'fake_pick_xiongmao') {
          workflowRun.value.actionStartedAt = Date.now()
          workflowRun.value.actionDurationSec = action.durationSec || 5
          workflowRun.value = { ...workflowRun.value }
          showNavMessage('', `动作链：正在执行第 ${index + 1} 步“拾取熊猫烟”（${workflowRun.value.actionDurationSec} 秒）...`)
          const actionData = await api.post('/api/actions/execute', {
            type: 'fake_pick_xiongmao',
            name: '拾取熊猫烟',
            duration_sec: workflowRun.value.actionDurationSec,
          })
          if (!actionData.ok) throw new Error(actionData.error || '假动作执行失败')
          workflowRun.value.completed[action.id] = true
          continue
        }
        throw new Error(`不支持的动作类型：${action.type}`)
      }
      const run = { ...workflowRun.value }
      run.running = false
      run.note = '动作链完成'
      workflowRun.value = run
      showNavMessage('', '动作链：<strong>已完成</strong>')
    } catch (err) {
      markWorkflowError(String(err.message || err))
      showNavMessage('bad', `动作链：<strong>执行失败</strong> ${err.message || err}`)
    }
  }

  async function stopNavigation() {
    navButtonsBusy.value = true
    showNavMessage('', '停止：正在取消底盘导航，并发送机械臂停止/复位指令...')
    try {
      const data = await api.post('/api/actions/stop_all', {})
      if (data.ok) {
        const phases = Array.isArray(data.arm_stop_phases) ? data.arm_stop_phases.join(' / ') : '--'
        showNavMessage('', `停止：<strong>已发送</strong> 底盘取消 + 机械臂 ${phases}`)
      } else {
        showNavMessage('bad', `停止：<strong>失败</strong> ${data.error || ''}`)
      }
      const run = { ...workflowRun.value }
      run.running = false
      run.note = '已停止'
      workflowRun.value = run
    } catch (err) {
      showNavMessage('bad', `导航：<strong>停止请求失败</strong> ${err}`)
    } finally {
      navButtonsBusy.value = false
      telemetry.tick()
    }
  }

  // ---- heading ----
  function setHeadingMode(enabled) {
    headingMode.value = Boolean(enabled)
  }

  function applyHeadingDegFromInput(rawValue) {
    if (!selectedWaypoints.value.length) {
      showNavMessage('bad', '导航：<strong>请先添加至少一个航点，再输入角度</strong>')
      return null
    }
    const raw = Number(rawValue)
    if (!Number.isFinite(raw)) {
      showNavMessage('bad', '导航：<strong>请输入有效角度</strong>')
      return null
    }
    const yaw = normalizeAngle((raw * Math.PI) / 180)
    manualHeadingDeg.value = Number(radToDeg(yaw).toFixed(3))
    setLastNavigationYaw(manualHeadingDeg.value)
    finalHeadingPoint.value = null
    setHeadingMode(false)
    showNavMessage('', `导航：已输入终点朝向 <strong>${manualHeadingDeg.value.toFixed(1)}°</strong>`)
    return manualHeadingDeg.value
  }

  // ---- map click / waypoint edits ----
  function handleMapClick(mapPoint, odomYawDeg) {
    if (workflowRun.value.running) {
      showNavMessage('bad', '动作链执行中：<strong>请先停止后再修改导航点</strong>')
      return { handled: false }
    }
    if (headingMode.value) {
      if (!selectedWaypoints.value.length) {
        showNavMessage('bad', '导航：<strong>请先添加终点，再设置朝向</strong>')
        setHeadingMode(false)
        return { handled: true }
      }
      finalHeadingPoint.value = { x: Number(mapPoint.x.toFixed(4)), y: Number(mapPoint.y.toFixed(4)) }
      manualHeadingDeg.value = null
      setHeadingMode(false)
      const targetYaw = computeTargetYaw(lastState())
      if (targetYaw) setLastNavigationYaw(targetYaw.yawDeg)
      showNavMessage('', `导航：已设置终点朝向 <strong>${targetYaw ? targetYaw.yawDeg.toFixed(1) : '--'}°</strong>`)
      return { handled: true, headingDeg: targetYaw ? targetYaw.yawDeg : null }
    }
    const yawDeg = odomYawDeg || 0
    addWorkflowAction({
      id: makeActionId('nav'),
      type: 'navigate',
      title: '导航',
      x: Number(mapPoint.x.toFixed(4)),
      y: Number(mapPoint.y.toFixed(4)),
      yawDeg: Number(radToDeg(normalizeAngle((yawDeg * Math.PI) / 180)).toFixed(3)),
    })
    finalHeadingPoint.value = null
    return { handled: true, poseFilled: { x: mapPoint.x, y: mapPoint.y, yawDeg } }
  }

  function undoWaypoint() {
    const lastNav = getLastNavigationAction()
    if (lastNav) removeWorkflowAction(lastNav.id)
    finalHeadingPoint.value = null
    manualHeadingDeg.value = null
    if (!selectedWaypoints.value.length) setHeadingMode(false)
  }

  function clearWaypoints() {
    workflowActions.value = workflowActions.value.filter(action => action.type !== 'navigate')
    saveWorkflowCache()
    syncSelectedWaypointsFromActions()
    finalHeadingPoint.value = null
    manualHeadingDeg.value = null
    setHeadingMode(false)
    resetWorkflowRun('待执行')
  }

  function toggleHeadingMode() {
    if (!selectedWaypoints.value.length) {
      showNavMessage('bad', '导航：<strong>请先添加至少一个航点，再设置朝向</strong>')
      return
    }
    setHeadingMode(!headingMode.value)
  }

  function clearHeading() {
    finalHeadingPoint.value = null
    manualHeadingDeg.value = null
    setHeadingMode(false)
  }

  return {
    // state
    workflowActions,
    selectedWaypoints,
    headingMode,
    finalHeadingPoint,
    manualHeadingDeg,
    editingActionId,
    draggedActionId,
    workflowRun,
    navButtonsBusy,
    navMode,
    navSpeedRatio,
    navStatusHtml,
    navStatusKind,
    // derived helpers
    computeTargetYaw,
    getWorkflowModules,
    getResolvedWorkflowActions,
    getSavedPointById: id => points.getById(id),
    actionTitle,
    actionDetail,
    armActionTitle,
    workflowStepStatus,
    workflowStepProgress,
    // mutations
    loadWorkflowCache,
    saveWorkflowCache,
    refreshLinkedWorkflowActions,
    syncSelectedWaypointsFromActions,
    clearWorkflowEditMode,
    commitWorkflowAction,
    addWorkflowAction,
    removeWorkflowAction,
    reorderWorkflowAction,
    setLastNavigationYaw,
    clearWorkflowActions,
    updateWorkflowProgress,
    updateNavigationStatus,
    showNavMessage,
    // run
    startNavigation,
    runWorkflow,
    stopNavigation,
    // heading & map
    setHeadingMode,
    toggleHeadingMode,
    clearHeading,
    applyHeadingDegFromInput,
    handleMapClick,
    undoWaypoint,
    clearWaypoints,
  }
})
