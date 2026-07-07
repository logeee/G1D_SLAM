<script setup>
import { computed, reactive, ref } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { usePointsStore } from '../stores/points.js'
import { useWorkflowStore } from '../stores/workflow.js'
import { normalizeAngle, radToDeg } from '../utils/format.js'

const telemetry = useTelemetryStore()
const points = usePointsStore()
const workflow = useWorkflowStore()

const { savedPoints, editingPointId, loadError } = storeToRefs(points)

const form = reactive({ name: '', yawDeg: '', x: '', y: '', note: '', actions: '[]' })
const message = reactive({ kind: '', text: '动作字段只保存，不执行；后续接机械臂时复用。' })

const pointsMeta = computed(() => {
  if (loadError.value) return 'load failed'
  return savedPoints.value.length ? `${savedPoints.value.length} saved` : 'waiting'
})

function setMessage(kind, text) {
  message.kind = kind || ''
  message.text = text
}

function setForm(point) {
  points.editingPointId = point?.id || null
  form.name = point?.name || ''
  form.x = point?.x !== undefined && point?.x !== null ? Number(point.x).toFixed(4) : ''
  form.y = point?.y !== undefined && point?.y !== null ? Number(point.y).toFixed(4) : ''
  form.yawDeg = point?.yaw_deg !== undefined && point?.yaw_deg !== null ? Number(point.yaw_deg).toFixed(1) : ''
  form.note = point?.note || ''
  form.actions = JSON.stringify(point?.actions || [], null, 2)
}

function clearForm() {
  setForm(null)
  form.actions = '[]'
  setMessage('', '动作字段只保存，不执行；后续接机械臂时复用。')
}

function parseActions() {
  const raw = form.actions.trim()
  if (!raw) return []
  let actions
  try {
    actions = JSON.parse(raw)
  } catch (err) {
    throw new Error(`动作 JSON 格式不对：${err.message}`)
  }
  if (!Array.isArray(actions)) throw new Error('动作 JSON 必须是数组，例如 [{"type":"pick"}]')
  return actions
}

function buildPayload(includeId = true) {
  const x = Number(form.x)
  const y = Number(form.y)
  const yawDeg = Number(form.yawDeg || 0)
  if (!Number.isFinite(x) || !Number.isFinite(y)) throw new Error('请填写有效的 X / Y 坐标')
  if (!Number.isFinite(yawDeg)) throw new Error('请填写有效的朝向角度')
  const payload = { name: form.name.trim(), x, y, yaw_deg: yawDeg, note: form.note, actions: parseActions() }
  if (includeId && editingPointId.value) payload.id = editingPointId.value
  return payload
}

async function reload() {
  try {
    await points.load()
    workflow.refreshLinkedWorkflowActions()
  } catch (err) {
    setMessage('bad', `点位列表读取失败：${err}`)
  }
}

async function recordCurrent() {
  let actions
  try {
    actions = parseActions()
  } catch (err) {
    setMessage('bad', err.message)
    return
  }
  setMessage('', '正在记录当前机器人位置...')
  try {
    const data = await points.recordCurrent({ name: form.name.trim(), note: form.note, actions })
    if (!data.ok) {
      setMessage('bad', `记录失败：${data.error || 'unknown error'}`)
      return
    }
    points.editingPointId = data.point.id
    await reload()
    const point = points.getById(editingPointId.value)
    if (point) setForm(point)
    setMessage('ok', `已记录当前位置：${data.point.name}`)
  } catch (err) {
    setMessage('bad', `记录失败：${err}`)
  }
}

async function save() {
  let payload
  try {
    payload = buildPayload(true)
  } catch (err) {
    setMessage('bad', err.message)
    return
  }
  try {
    const data = await points.upsert(payload)
    if (!data.ok) {
      setMessage('bad', `保存失败：${data.error || 'unknown error'}`)
      return
    }
    points.editingPointId = data.point.id
    await reload()
    const point = points.getById(editingPointId.value)
    if (point) setForm(point)
    setMessage('ok', `已保存点位：${data.point.name}`)
  } catch (err) {
    setMessage('bad', `保存失败：${err}`)
  }
}

async function del() {
  if (!editingPointId.value) {
    setMessage('bad', '请先在列表里选中一个点位')
    return
  }
  if (!window.confirm('确定删除这个点位吗？')) return
  try {
    const data = await points.remove(editingPointId.value)
    if (!data.ok) {
      setMessage('bad', `删除失败：${data.error || 'unknown error'}`)
      return
    }
    clearForm()
    await reload()
    setMessage('ok', '点位已删除')
  } catch (err) {
    setMessage('bad', `删除失败：${err}`)
  }
}

function addToNav() {
  let payload
  try {
    payload = buildPayload(false)
  } catch (err) {
    setMessage('bad', err.message)
    return
  }
  workflow.addWorkflowAction({
    id: `nav-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`,
    type: 'navigate',
    title: payload.name ? `导航到 ${payload.name}` : '导航',
    pointId: editingPointId.value || null,
    pointName: payload.name || '',
    x: Number(payload.x.toFixed(4)),
    y: Number(payload.y.toFixed(4)),
    yawDeg: Number(radToDeg(normalizeAngle((payload.yaw_deg * Math.PI) / 180)).toFixed(3)),
  })
  workflow.clearHeading()
  setMessage('ok', `已加入动作链：${payload.name || '导航'}，朝向 ${Number(payload.yaw_deg).toFixed(1)}°`)
}

function selectPoint(point) {
  setForm(point)
  setMessage('', `正在编辑：${point.name || 'Point'}`)
}
</script>

<template>
  <section>
    <div class="panel-head">
      <h2>点位库 / 动作预留</h2>
      <span class="meta">{{ pointsMeta }}</span>
    </div>
    <div class="point-panel-body">
      <div class="point-form">
        <div class="point-grid">
          <div>
            <label>名称</label>
            <input v-model="form.name" type="text" placeholder="例如：货架 A 点" />
          </div>
          <div>
            <label>朝向 deg</label>
            <input v-model="form.yawDeg" type="number" step="0.1" placeholder="0" />
          </div>
          <div>
            <label>X m</label>
            <input v-model="form.x" type="number" step="0.001" placeholder="地图 X" />
          </div>
          <div>
            <label>Y m</label>
            <input v-model="form.y" type="number" step="0.001" placeholder="地图 Y" />
          </div>
        </div>
        <div>
          <label>备注</label>
          <textarea v-model="form.note" placeholder="可写用途、货架、调试说明"></textarea>
        </div>
        <div>
          <label>动作 JSON（预留，不执行）</label>
          <textarea v-model="form.actions" spellcheck="false"></textarea>
        </div>
        <div class="point-actions">
          <button class="primary" @click="recordCurrent">记录当前位置</button>
          <button @click="clearForm">新建/清空</button>
          <button @click="save">保存点位</button>
          <button @click="addToNav">加入导航</button>
          <button class="danger" @click="del">删除</button>
        </div>
        <div class="point-message" :class="message.kind">{{ message.text }}</div>
      </div>
      <div class="point-list">
        <template v-if="savedPoints.length">
          <div
            v-for="point in savedPoints"
            :key="point.id"
            class="point-item"
            :class="{ active: point.id === editingPointId }"
            @click="selectPoint(point)"
          >
            <div class="point-title">
              <span>{{ point.name || 'Point' }}</span>
              <span>{{ Number(point.yaw_deg || 0).toFixed(1) }}°</span>
            </div>
            <div class="point-detail">
              x={{ Number(point.x).toFixed(3) }}m, y={{ Number(point.y).toFixed(3) }}m · {{ point.source || 'manual' }} · actions
              {{ Array.isArray(point.actions) ? point.actions.length : 0 }}
            </div>
            <div v-if="point.note" class="point-detail">{{ point.note }}</div>
          </div>
        </template>
        <div v-else class="point-item">
          <div class="point-title"><span>暂无保存点位</span></div>
          <div class="point-detail">可以先点击“记录当前位置”，也可以手动填写 X/Y 后保存。</div>
        </div>
      </div>
    </div>
  </section>
</template>
