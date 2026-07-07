import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'
import { useTelemetryStore } from './telemetry.js'

// Relocalization status line + anchor save + recover run.
export const useRelocalizationStore = defineStore('relocalization', () => {
  const telemetry = useTelemetryStore()

  const statusHtml = ref('重定位：等待保存开机基准')
  const statusKind = ref('')
  const radius = ref(0.6)
  const movement = ref('NO_MOVE')
  const runBusy = ref(false)

  function setMessage(kind, text) {
    statusKind.value = kind || ''
    statusHtml.value = text
  }

  function renderStatus(data) {
    if (!data?.ok) {
      setMessage('bad', `重定位：状态读取失败 ${data?.error || ''}`)
      return
    }
    const anchor = data.anchor
    const basic = data.robot_basic_state
    const last = data.last_relocalization_command
    const anchorText = anchor
      ? `基准 x=${Number(anchor.x).toFixed(3)}m, y=${Number(anchor.y).toFixed(3)}m, yaw=${Number(anchor.yaw_deg || 0).toFixed(1)}°`
      : '未保存开机基准'
    const qualityText = basic ? `定位 ${basic.is_localization_enabled ? 'ON' : 'OFF'} / ${basic.localization_quality}` : '定位 --'
    const lastText = last ? `上次 ${last.movement || '--'} 半径 ${Number(last.search_radius_m || 0).toFixed(2)}m #${last.seq || ''}` : '未执行'
    setMessage(anchor ? '' : 'warn', `重定位：${anchorText}　${qualityText}　${lastText}`)
  }

  async function loadStatus() {
    try {
      const data = await api.getRelocalizationStatus()
      renderStatus(data)
    } catch (err) {
      setMessage('bad', `重定位：状态读取失败 ${err}`)
    }
  }

  async function saveAnchor() {
    setMessage('', '重定位：正在保存当前 odom 为开机基准...')
    const data = await api.post('/api/relocalization/save_anchor', { source: 'dashboard_button' })
    if (!data.ok) {
      setMessage('bad', `重定位：保存失败 ${data.error || ''}`)
      return
    }
    renderStatus({
      ok: true,
      anchor: data.anchor,
      robot_basic_state: telemetry.state?.navigation?.robot_basic_state,
      last_relocalization_command: telemetry.state?.navigation?.last_relocalization_command,
    })
  }

  async function runRelocalization() {
    const r = Number(radius.value || 0.6)
    if (!Number.isFinite(r) || r <= 0) {
      setMessage('bad', '重定位：请输入有效搜索半径')
      return
    }
    setMessage('', '重定位：正在发送 set_pose + recover_localization...')
    runBusy.value = true
    try {
      const data = await api.post('/api/relocalization/run', {
        search_radius_m: r,
        movement: movement.value,
        max_time_ms: 8000,
        set_pose: true,
        enable_localization: true,
      })
      if (!data.ok) {
        setMessage('bad', `重定位：执行失败 ${data.error || ''}`)
        return
      }
      setMessage('warn', `重定位：已发送命令，方式 ${movement.value}，半径 ${r.toFixed(2)}m；等待 Slamware 更新定位质量...`)
      setTimeout(loadStatus, 1200)
      setTimeout(loadStatus, 4000)
      setTimeout(loadStatus, 8500)
    } finally {
      runBusy.value = false
    }
  }

  return { statusHtml, statusKind, radius, movement, runBusy, loadStatus, saveAnchor, runRelocalization }
})
