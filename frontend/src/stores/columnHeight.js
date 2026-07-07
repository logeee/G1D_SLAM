import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'
import { useWorkflowStore } from './workflow.js'

// G1D lift column height readout + status, ported from the legacy lift-height code.
export const useColumnHeightStore = defineStore('columnHeight', () => {
  const lastLiftHeight = ref(null)
  const refreshInFlight = ref(false)
  const statusText = ref('当前立柱高度：--')
  // Readout shown in the map panel "立柱高度" metric.
  const physicalHeightText = ref('--')
  const physicalHeightTitle = ref('')
  const physicalMaxM = ref(null)

  function updateReadout(data) {
    if (!data || data.ok === false || !Number.isFinite(Number(data.physical_height_m))) {
      physicalHeightText.value = '--'
      physicalHeightTitle.value = data?.error || ''
      return
    }
    const physical = Number(data.physical_height_m)
    const max = Number.isFinite(Number(data.physical_max_m))
      ? Number(data.physical_max_m)
      : Number.isFinite(Number(data.full_travel_m))
        ? Number(data.full_travel_m)
        : null
    physicalHeightText.value = `${physical.toFixed(3)} m`
    physicalHeightTitle.value = max !== null ? `physical ${physical.toFixed(3)} m / range 0~${max.toFixed(3)} m` : `physical ${physical.toFixed(3)} m`
  }

  function updateStatus(data) {
    updateReadout(data)
    if (!data || data.ok === false) {
      statusText.value = `当前立柱高度：读取失败 ${data?.error || ''}`
      return
    }
    lastLiftHeight.value = data
    const physical = Number.isFinite(Number(data.physical_height_m)) ? Number(data.physical_height_m).toFixed(3) : '--'
    const raw = Number.isFinite(Number(data.hispeed_y_m)) ? Number(data.hispeed_y_m).toFixed(3) : '--'
    const offset = Number.isFinite(Number(data.lift_offset_m)) ? Number(data.lift_offset_m).toFixed(3) : '--'
    const max = Number.isFinite(Number(data.physical_max_m))
      ? Number(data.physical_max_m).toFixed(3)
      : Number.isFinite(Number(data.full_travel_m))
        ? Number(data.full_travel_m).toFixed(3)
        : '--'
    const age = Number.isFinite(Number(data.data_age_sec)) ? `${Number(data.data_age_sec).toFixed(1)}s` : '--'
    statusText.value = `当前立柱物理高度 ${physical} m（raw=${raw} m，offset=${offset} m，范围 0~${max} m，age=${age}）`
    if (Number.isFinite(Number(data.physical_max_m))) physicalMaxM.value = Number(data.physical_max_m)
  }

  async function refresh({ quiet = false } = {}) {
    if (refreshInFlight.value) return lastLiftHeight.value
    refreshInFlight.value = true
    const workflow = useWorkflowStore()
    try {
      const data = await api.getLiftHeight()
      updateStatus(data)
      if (!quiet) {
        workflow.showNavMessage(
          data.ok ? '' : 'bad',
          data.ok ? `立柱：当前物理高度 <strong>${Number(data.physical_height_m || 0).toFixed(3)}m</strong>` : `立柱：<strong>高度读取失败</strong> ${data.error || ''}`,
        )
      }
      return data
    } catch (err) {
      const data = { ok: false, error: String(err) }
      updateStatus(data)
      if (!quiet) workflow.showNavMessage('bad', `立柱：<strong>高度读取失败</strong> ${err}`)
      return data
    } finally {
      refreshInFlight.value = false
    }
  }

  // Returns the physical height to fill into a form, or null.
  async function fillCurrent() {
    const workflow = useWorkflowStore()
    const data = lastLiftHeight.value?.ok ? lastLiftHeight.value : await refresh({ quiet: true })
    if (!data?.ok || !Number.isFinite(Number(data.physical_height_m))) {
      workflow.showNavMessage('bad', `立柱：<strong>没有可用的当前物理高度</strong> ${data?.error || ''}`)
      return null
    }
    const value = Number(data.physical_height_m)
    workflow.showNavMessage('', `立柱：已填当前物理高度 <strong>${value.toFixed(3)}m</strong>`)
    return value
  }

  return { lastLiftHeight, statusText, physicalHeightText, physicalHeightTitle, physicalMaxM, refresh, fillCurrent }
})
