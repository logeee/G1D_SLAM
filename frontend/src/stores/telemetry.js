import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'

// Central polling store. Replaces the legacy `setInterval(tick, 500)` +
// global `lastState`. Components read `state` reactively; canvas components
// draw imperatively from `state` (reactivity drives layout/readouts only).
export const useTelemetryStore = defineStore('telemetry', () => {
  const state = ref(null)
  const online = ref(false)
  const statusText = ref('connecting')
  const lastError = ref('')

  let timer = null

  async function tick() {
    try {
      const snap = await api.getState()
      state.value = snap
      online.value = true
      const uptime = Number(snap?.uptime_s)
      statusText.value = Number.isFinite(uptime) ? `online, uptime ${uptime.toFixed(1)}s` : 'online'
      lastError.value = ''
    } catch (err) {
      online.value = false
      statusText.value = `offline: ${err}`
      lastError.value = String(err)
    }
  }

  function startPolling(intervalMs = 500) {
    if (timer) return
    tick()
    timer = setInterval(tick, intervalMs)
  }

  function stopPolling() {
    if (timer) {
      clearInterval(timer)
      timer = null
    }
  }

  return { state, online, statusText, lastError, tick, startPolling, stopPolling }
})
