import { defineStore } from 'pinia'
import { ref, shallowRef, markRaw } from 'vue'
import { api } from '../api/client.js'

// Central polling store. Replaces the legacy `setInterval(tick, 500)` +
// global `lastState`. Components read `state` reactively; canvas components
// draw imperatively from `state` (reactivity drives layout/readouts only).
//
// IMPORTANT: `state` is a shallowRef holding a markRaw snapshot. /api/state can
// be ~1.25 MB and contains map.data (a ~400k-int occupancy grid) + track. A
// plain ref would make Vue deep-proxy that whole array on every 500 ms poll,
// pinning the main thread. We replace the snapshot wholesale each tick and never
// mutate nested fields, so shallow + raw reactivity is both correct and fast.
export const useTelemetryStore = defineStore('telemetry', () => {
  const state = shallowRef(null)
  const online = ref(false)
  const statusText = ref('connecting')
  const lastError = ref('')

  let timer = null

  async function tick() {
    try {
      const snap = await api.getState()
      state.value = markRaw(snap)
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

  // One-off fetch used by the workflow run engine (legacy fetchStateOnce).
  async function fetchOnce() {
    const snap = await api.getState()
    if (snap?.ok === false) throw new Error(snap?.error || 'state request failed')
    state.value = markRaw(snap)
    return snap
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

  return { state, online, statusText, lastError, tick, fetchOnce, startPolling, stopPolling }
})
