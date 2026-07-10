<script setup>
import { computed, ref } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useWorkflowStore } from '../stores/workflow.js'
import { usePointsStore } from '../stores/points.js'
import { useColumnHeightStore } from '../stores/columnHeight.js'
import { useDialogStore } from '../stores/dialog.js'
import { normalizeAngle, radToDeg, clampSpeedRatio } from '../utils/format.js'

const telemetry = useTelemetryStore()
const workflow = useWorkflowStore()
const points = usePointsStore()
const columnHeight = useColumnHeightStore()
const dialog = useDialogStore()

const { state } = storeToRefs(telemetry)
const { workflowRun, editingActionId, navButtonsBusy, savedChains } = storeToRefs(workflow)
const { savedPoints } = storeToRefs(points)
const { statusText: columnStatusText } = storeToRefs(columnHeight)

const running = computed(() => workflowRun.value.running)
const busy = computed(() => navButtonsBusy.value || running.value)

// ---- action-chain library (save/load/delete/copy to disk) ----
function chainSelectItems() {
  return savedChains.value.map(chain => ({
    value: chain.id,
    label: `${chain.name}（${chain.count} 个动作）`,
  }))
}

// Refresh the library, then let the user pick a saved chain via the modal select.
// Returns the chosen chain id, or null if none / cancelled.
async function pickSavedChain(message, title, opts = {}) {
  await workflow.loadChainLibrary()
  if (!savedChains.value.length) {
    await dialog.alert('还没有已保存的动作链，请先“保存当前动作链”。', { title })
    return null
  }
  return dialog.select(message, chainSelectItems(), { title, ...opts })
}

async function onSaveChain() {
  if (busy.value) return
  if (!workflow.workflowActions.length) {
    workflow.showNavMessage('bad', '动作链：<strong>当前没有动作可保存</strong>')
    return
  }
  const defaultName = `动作链 ${new Date().toLocaleString('zh-CN', { hour12: false })}`
  const name = await dialog.prompt('给这条动作链起个名字（同名会覆盖）', defaultName, { title: '保存当前动作链' })
  if (name === null) return
  const trimmed = String(name).trim()
  if (!trimmed) {
    workflow.showNavMessage('bad', '动作链：<strong>名字不能为空</strong>')
    return
  }
  try {
    const data = await workflow.saveChainToDisk(trimmed)
    if (data.ok) workflow.showNavMessage('', `动作链：已保存 <strong>${data.chain?.name || trimmed}</strong> 到本地`)
    else workflow.showNavMessage('bad', `动作链：<strong>保存失败</strong> ${data.error || ''}`)
  } catch (err) {
    workflow.showNavMessage('bad', `动作链：<strong>保存请求失败</strong> ${err}`)
  }
}

async function onLoadChain() {
  if (busy.value) return
  const id = await pickSavedChain('选择要加载的动作链（会替换当前动作）', '加载动作链', { confirmText: '加载' })
  if (!id) return
  await workflow.loadChainFromDisk(id)
}

async function onDeleteChain() {
  if (busy.value) return
  const id = await pickSavedChain('选择要删除的动作链', '删除动作链', { confirmText: '下一步', danger: true })
  if (!id) return
  const chain = savedChains.value.find(item => item.id === id)
  const name = chain?.name || '该动作链'
  const ok = await dialog.confirm(`确定删除“${name}”吗？此操作不可恢复。`, { title: '删除动作链', danger: true, confirmText: '删除' })
  if (!ok) return
  try {
    const data = await workflow.deleteChain(id)
    if (data.ok) workflow.showNavMessage('', `动作链：已删除 <strong>${name}</strong>`)
    else workflow.showNavMessage('bad', `动作链：<strong>删除失败</strong> ${data.error || ''}`)
  } catch (err) {
    workflow.showNavMessage('bad', `动作链：<strong>删除请求失败</strong> ${err}`)
  }
}

async function onCopyChain() {
  if (busy.value) return
  const id = await pickSavedChain('选择要复制的动作链', '复制动作链', { confirmText: '下一步' })
  if (!id) return
  const chain = savedChains.value.find(item => item.id === id)
  if (!chain) return
  const newName = await dialog.prompt('复制为新名称', `${chain.name} 副本`, { title: '复制动作链' })
  if (newName === null) return
  const trimmed = String(newName).trim()
  if (!trimmed) {
    workflow.showNavMessage('bad', '动作链：<strong>名字不能为空</strong>')
    return
  }
  try {
    const data = await workflow.copyChain(id, trimmed)
    if (data.ok) workflow.showNavMessage('', `动作链：已复制为 <strong>${data.chain?.name || trimmed}</strong>`)
    else workflow.showNavMessage('bad', `动作链：<strong>复制失败</strong> ${data.error || ''}`)
  } catch (err) {
    workflow.showNavMessage('bad', `动作链：<strong>复制请求失败</strong> ${err}`)
  }
}

// ---- action builder local state ----
const newActionType = ref('navigate')
const newActionPointSelect = ref('')
const newActionX = ref('')
const newActionY = ref('')
const newActionYawDeg = ref('')
const newActionSpeedRatio = ref(1)
const newArmTargetObject = ref('XiongMao')
const newArmTimeoutSec = ref(120)
const newColumnTargetHeightM = ref(0)
const newColumnTimeoutSec = ref(30)

const isNav = computed(() => newActionType.value === 'navigate')
const isArm = computed(() => newActionType.value.startsWith('arm_'))
const isArmPick = computed(() => newActionType.value === 'arm_pick')
const isColumn = computed(() => newActionType.value === 'column_height')

const addBtnLabel = computed(() => (editingActionId.value ? '保存动作' : '增加动作'))

const workflowMeta = computed(() => {
  void state.value
  const run = workflowRun.value
  const modules = workflow.getWorkflowModules()
  const doneCount = modules.filter(m => run.completed[m.id]).length
  return run.running ? `执行中 ${Math.min(run.currentIndex + 1, modules.length)}/${modules.length}` : `${doneCount}/${modules.length} 完成`
})

const steps = computed(() => {
  void state.value // recompute each telemetry tick so progress advances
  const modules = workflow.getWorkflowModules()
  return modules.map((m, index) => {
    const status = workflow.workflowStepStatus(m, index)
    const progress = workflow.workflowStepProgress(m, index)
    const badge = status === 'done' ? '完成' : status === 'running' ? '进行中' : status === 'error' ? '异常' : '等待'
    return { module: m, index, status, progress, badge, title: workflow.actionTitle(m, index), detail: workflow.actionDetail(m) }
  })
})

function onTypeChange() {
  if (isColumn.value) columnHeight.refresh({ quiet: true })
}

function onPointSelected() {
  const point = points.getById(newActionPointSelect.value)
  if (!point) return
  newActionX.value = Number(point.x).toFixed(4)
  newActionY.value = Number(point.y).toFixed(4)
  newActionYawDeg.value = Number(point.yaw_deg || 0).toFixed(1)
  workflow.showNavMessage('', `动作链：已选择点位库 <strong>${point.name || 'Point'}</strong>`)
}

async function fillColumnCurrent() {
  const value = await columnHeight.fillCurrent()
  if (value !== null && value !== undefined) newColumnTargetHeightM.value = Number(value.toFixed(3))
}

function fillCurrentPose() {
  if (!state.value?.odom) {
    workflow.showNavMessage('bad', '动作链：<strong>当前没有 odom，不能填当前位置</strong>')
    return
  }
  newActionPointSelect.value = ''
  newActionX.value = Number(state.value.odom.x).toFixed(4)
  newActionY.value = Number(state.value.odom.y).toFixed(4)
  newActionYawDeg.value = Number(state.value.odom.yaw_deg || 0).toFixed(1)
}

function makeActionId(prefix) {
  return `${prefix}-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`
}

function addActionFromBuilder() {
  const type = newActionType.value
  if (type === 'navigate') {
    const selectedPoint = points.getById(newActionPointSelect.value || '')
    const x = selectedPoint ? Number(selectedPoint.x) : Number(newActionX.value)
    const y = selectedPoint ? Number(selectedPoint.y) : Number(newActionY.value)
    const rawYaw = selectedPoint ? selectedPoint.yaw_deg : newActionYawDeg.value
    const yawDeg = rawYaw === '' || rawYaw === null || rawYaw === undefined ? state.value?.odom?.yaw_deg || 0 : Number(rawYaw)
    const speedRatio = clampSpeedRatio(newActionSpeedRatio.value, 1)
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      workflow.showNavMessage('bad', '动作链：<strong>导航动作需要有效的 X / Y</strong>')
      return
    }
    if (!Number.isFinite(yawDeg)) {
      workflow.showNavMessage('bad', '动作链：<strong>导航动作需要有效的 yaw</strong>')
      return
    }
    const commitMode = workflow.commitWorkflowAction({
      id: makeActionId('nav'),
      type: 'navigate',
      title: selectedPoint ? `导航到 ${selectedPoint.name || '点位'}` : '导航',
      pointId: selectedPoint?.id || null,
      pointName: selectedPoint?.name || '',
      x: Number(x.toFixed(4)),
      y: Number(y.toFixed(4)),
      yawDeg: Number(radToDeg(normalizeAngle((yawDeg * Math.PI) / 180)).toFixed(3)),
      speedRatio: Number(speedRatio.toFixed(3)),
    })
    const sourceText = selectedPoint ? `（点位库：${selectedPoint.name || 'Point'}）` : ''
    workflow.showNavMessage('', `动作链：已${commitMode === 'edited' ? '保存' : '增加'}导航动作${sourceText} x=${x.toFixed(3)}, y=${y.toFixed(3)}`)
    return
  }
  if (type.startsWith('arm_')) {
    const phaseMap = { arm_pick: 'PICK', arm_place: 'PLACE', arm_reset: 'RESET' }
    const phase = phaseMap[type]
    const targetObject = phase === 'PICK' ? newArmTargetObject.value : ''
    const timeoutSec = Number(newArmTimeoutSec.value || 120)
    if (!phase) {
      workflow.showNavMessage('bad', '动作链：<strong>未知机械臂任务类型</strong>')
      return
    }
    if (phase === 'PICK' && !targetObject) {
      workflow.showNavMessage('bad', '动作链：<strong>机械臂抓取需要选择目标标签</strong>')
      return
    }
    if (!Number.isFinite(timeoutSec) || timeoutSec <= 0) {
      workflow.showNavMessage('bad', '动作链：<strong>机械臂任务需要有效超时时间</strong>')
      return
    }
    const action = {
      id: makeActionId('arm'),
      type: 'arm_task',
      phase,
      targetObject,
      timeoutSec: Math.max(1, Math.min(600, Number(timeoutSec.toFixed(1)))),
    }
    action.title = workflow.armActionTitle(action)
    const commitMode = workflow.commitWorkflowAction(action)
    workflow.showNavMessage('', `动作链：已${commitMode === 'edited' ? '保存' : '增加'}“${action.title}”`)
    return
  }
  if (type === 'column_height') {
    const targetPhysicalHeightM = Number(newColumnTargetHeightM.value)
    const timeoutSec = Number(newColumnTimeoutSec.value || 30)
    const lift = columnHeight.lastLiftHeight
    const maxPhysical = Number(lift?.physical_max_m ?? lift?.full_travel_m ?? 0.427)
    if (!Number.isFinite(targetPhysicalHeightM)) {
      workflow.showNavMessage('bad', '动作链：<strong>立柱升降需要有效物理目标高度</strong>')
      return
    }
    if (targetPhysicalHeightM < -0.002 || targetPhysicalHeightM > maxPhysical + 0.002) {
      workflow.showNavMessage('bad', `动作链：<strong>立柱物理高度范围是 0.000 ~ ${maxPhysical.toFixed(3)} m</strong>`)
      return
    }
    if (!Number.isFinite(timeoutSec) || timeoutSec <= 0) {
      workflow.showNavMessage('bad', '动作链：<strong>立柱升降需要有效超时时间</strong>')
      return
    }
    const action = {
      id: makeActionId('column'),
      type: 'column_height',
      title: '立柱升降',
      targetPhysicalHeightM: Number(targetPhysicalHeightM.toFixed(4)),
      timeoutSec: Math.max(1, Math.min(180, Number(timeoutSec.toFixed(1)))),
    }
    const commitMode = workflow.commitWorkflowAction(action)
    workflow.showNavMessage('', `动作链：已${commitMode === 'edited' ? '保存' : '增加'}“立柱升降” 物理高度=${action.targetPhysicalHeightM.toFixed(3)}m`)
    return
  }
  const commitMode = workflow.commitWorkflowAction({ id: makeActionId('act'), type: 'fake_pick_xiongmao', title: '拾取熊猫烟', durationSec: 5 })
  workflow.showNavMessage('', `动作链：已${commitMode === 'edited' ? '保存' : '增加'}动作“拾取熊猫烟”`)
}

function editAction(actionId) {
  if (running.value) return
  const action = workflow.workflowActions.find(item => item.id === actionId)
  if (!action) return
  const resolved = workflow.getResolvedWorkflowActions().find(item => item.id === actionId) || action
  workflow.editingActionId = actionId
  let type = resolved.type
  if (resolved.type === 'arm_task') {
    const phase = String(resolved.phase || '').toUpperCase()
    type = phase === 'PICK' ? 'arm_pick' : phase === 'PLACE' ? 'arm_place' : 'arm_reset'
  }
  newActionType.value = type
  onTypeChange()
  if (type === 'navigate') {
    newActionPointSelect.value = resolved.pointId || ''
    newActionX.value = resolved.x !== undefined ? Number(resolved.x).toFixed(4) : ''
    newActionY.value = resolved.y !== undefined ? Number(resolved.y).toFixed(4) : ''
    newActionYawDeg.value = resolved.yawDeg !== undefined ? Number(resolved.yawDeg).toFixed(1) : ''
    newActionSpeedRatio.value = Number(clampSpeedRatio(resolved.speedRatio, 1).toFixed(2))
  } else if (type.startsWith('arm_')) {
    newArmTargetObject.value = resolved.targetObject || resolved.target_object || 'XiongMao'
    newArmTimeoutSec.value = Number(resolved.timeoutSec || 120)
  } else if (type === 'column_height') {
    const physicalTarget = resolved.targetPhysicalHeightM ?? resolved.target_physical_height_m ?? resolved.targetHeightM ?? 0
    newColumnTargetHeightM.value = Number(Number(physicalTarget || 0).toFixed(3))
    newColumnTimeoutSec.value = Number(resolved.timeoutSec || 30)
    columnHeight.refresh({ quiet: true })
  }
  const index = workflow.workflowActions.findIndex(item => item.id === actionId)
  workflow.showNavMessage('', `动作链：正在编辑第 <strong>${index + 1}</strong> 个动作，修改后点“保存动作”`)
}

function resetState() {
  workflow.clearWorkflowEditMode()
  // resetWorkflowRun via store
  workflow.updateWorkflowProgress(state.value)
  workflow.workflowRun = { ...workflow.workflowRun, running: false, note: '待执行', error: '', completed: {}, currentIndex: -1, mode: 'idle' }
}

// ---- drag & drop ----
function onDragStart(ev, id) {
  workflow.draggedActionId = id
  ev.dataTransfer.effectAllowed = 'move'
  ev.dataTransfer.setData('text/plain', id)
}
function onDrop(ev, targetId) {
  ev.preventDefault()
  const sourceId = workflow.draggedActionId || ev.dataTransfer.getData('text/plain')
  workflow.reorderWorkflowAction(sourceId, targetId)
}
function onDragEnd() {
  workflow.draggedActionId = null
}
</script>

<template>
  <aside class="workflow-panel">
    <div class="workflow-head">
      <strong>动作链</strong>
      <span class="meta">{{ workflowMeta }}</span>
    </div>
    <div class="workflow-control-row">
      <button class="primary" :disabled="busy" @click="workflow.runWorkflow()">执行动作链</button>
      <button :disabled="busy" @click="workflow.startNavigation()">仅执行导航</button>
      <button class="danger" @click="workflow.stopNavigation()">停止</button>
      <button :disabled="busy" @click="workflow.clearWorkflowActions()">清空动作</button>
    </div>
    <div class="workflow-control-row">
      <button :disabled="busy" @click="onSaveChain">保存当前动作链</button>
      <button :disabled="busy" @click="onLoadChain">加载动作链</button>
      <button class="danger" :disabled="busy" @click="onDeleteChain">删除</button>
      <button :disabled="busy" @click="onCopyChain">复制</button>
    </div>
    <div class="workflow-actions">
      <div class="action-builder">
        <label>动作类型
          <select v-model="newActionType" :disabled="busy" @change="onTypeChange">
            <option value="navigate">导航</option>
            <option value="arm_pick">机械臂抓取</option>
            <option value="arm_place">机械臂放置</option>
            <option value="arm_reset">机械臂复位</option>
            <option value="column_height">立柱升降</option>
          </select>
        </label>
        <label v-show="isNav" class="nav-action-field">点位库
          <select v-model="newActionPointSelect" :disabled="busy" @change="onPointSelected">
            <option value="">手动输入 / 地图点选</option>
            <option v-for="p in savedPoints" :key="p.id" :value="p.id">
              {{ p.name || 'Point' }} ({{ Number(p.x).toFixed(3) }}, {{ Number(p.y).toFixed(3) }}, {{ Number(p.yaw_deg || 0).toFixed(1) }}°)
            </option>
          </select>
        </label>
        <label v-show="isNav" class="nav-action-field">X m
          <input v-model="newActionX" type="number" step="0.001" placeholder="地图 X" :disabled="busy" />
        </label>
        <label v-show="isNav" class="nav-action-field">Y m
          <input v-model="newActionY" type="number" step="0.001" placeholder="地图 Y" :disabled="busy" />
        </label>
        <label v-show="isNav" class="nav-action-field">Yaw deg
          <input v-model="newActionYawDeg" type="number" step="0.1" placeholder="当前" :disabled="busy" />
        </label>
        <label v-show="isNav" class="nav-action-field">导航速度
          <input v-model.number="newActionSpeedRatio" type="number" step="0.05" min="0.05" max="1" :disabled="busy" />
        </label>
        <label v-show="isArm" class="arm-action-field">抓取目标
          <select v-model="newArmTargetObject" :disabled="busy || !isArmPick">
            <option value="XiongMao">XiongMao / 熊猫烟</option>
            <option value="Xizi_Liqun">Xizi_Liqun / 西子利群</option>
          </select>
        </label>
        <label v-show="isArm" class="arm-action-field">超时 s
          <input v-model.number="newArmTimeoutSec" type="number" step="1" min="1" max="600" :disabled="busy" />
        </label>
        <label v-show="isColumn" class="column-action-field">目标物理高度 m
          <input v-model.number="newColumnTargetHeightM" type="number" step="0.001" min="0" max="0.427" :disabled="busy" />
        </label>
        <label v-show="isColumn" class="column-action-field">超时 s
          <input v-model.number="newColumnTimeoutSec" type="number" step="1" min="1" max="180" :disabled="busy" />
        </label>
        <div v-show="isColumn" class="column-action-field column-height-status">{{ columnStatusText }}</div>
        <div v-show="isColumn" class="column-action-field column-height-buttons">
          <button type="button" :disabled="busy" @click="columnHeight.refresh()">刷新立柱高度</button>
          <button type="button" :disabled="busy" @click="fillColumnCurrent">填当前高度</button>
        </div>
        <div v-show="isColumn" class="column-action-field workflow-detail">
          这里填写真实物理高度 0.000 ~ 0.427 m；执行时后端会按当前 offset 自动换算为 raw SDK 目标。
        </div>
      </div>
      <div class="action-builder-actions">
        <button v-show="isNav" :disabled="busy" @click="fillCurrentPose">填当前位置</button>
        <button class="primary" :disabled="busy" @click="addActionFromBuilder">{{ addBtnLabel }}</button>
        <button :disabled="busy" @click="resetState">重置状态</button>
      </div>
      <div class="workflow-detail">
        导航动作可以从点位库选，也可以手动填位姿或直接点地图；机械臂抓取会把目标标签发给手臂模块；立柱升降会调用 G1D 高度控制原始动作。动作卡片可拖拽排序。
      </div>
    </div>
    <div class="workflow-list">
      <template v-if="steps.length">
        <div
          v-for="step in steps"
          :key="step.module.id"
          class="workflow-step"
          :class="step.status"
          :draggable="!running"
          :style="{ '--progress': step.progress + '%' }"
          @dragstart="onDragStart($event, step.module.id)"
          @dragover.prevent
          @drop="onDrop($event, step.module.id)"
          @dragend="onDragEnd"
        >
          <div class="workflow-step-content">
            <div class="workflow-title">
              <span class="workflow-title-left"><span class="drag-handle">☰</span><span class="workflow-pulse"></span>{{ step.title }}</span>
              <span class="workflow-actions-inline">
                <span class="workflow-badge">{{ step.badge }}</span>
                <button v-if="!running" class="workflow-edit" @click.stop="editAction(step.module.id)">编辑</button>
                <button v-if="!running" class="workflow-delete" @click.stop="workflow.removeWorkflowAction(step.module.id)">删除</button>
              </span>
            </div>
            <div class="workflow-detail">{{ step.detail }}</div>
            <div class="workflow-progress-text">进度 {{ step.progress }}%</div>
          </div>
        </div>
      </template>
      <div v-else class="workflow-step">
        <div class="workflow-step-content">
          <div class="workflow-title">暂无动作模块</div>
          <div class="workflow-detail">点击“增加动作”，或直接在地图上点击快速增加导航动作。</div>
        </div>
      </div>
    </div>
  </aside>
</template>
