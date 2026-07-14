<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { drawCloud } from '../utils/cloudDraw.js'
import { fmt } from '../utils/format.js'

const telemetry = useTelemetryStore()
const { state } = storeToRefs(telemetry)

const canvasEl = ref(null)
const view = reactive({ yaw: -0.75, pitch: 0.65 })
let dragging = false
let last = { x: 0, y: 0 }
let observer = null

const cloudMeta = computed(() => {
  const cloud = state.value?.point_cloud
  return cloud && cloud.points?.length ? `${cloud.sampled_points}/${cloud.total_points} points, age ${fmt(state.value.freshness_s?.point_cloud, 2, 's')}` : 'waiting'
})
const topic = computed(() => {
  const t = state.value?.point_cloud?.topic
  return t ? t.split('/').filter(Boolean).slice(-1)[0] : '--'
})
const pointsText = computed(() => {
  const c = state.value?.point_cloud
  return c ? `${c.sampled_points}/${c.total_points}` : '--'
})
const frame = computed(() => state.value?.point_cloud?.frame_id || '--')
const age = computed(() => fmt(state.value?.freshness_s?.point_cloud, 2, 's'))

function render() {
  if (canvasEl.value) drawCloud(canvasEl.value, state.value, view)
}

function onPointerDown(ev) {
  dragging = true
  last = { x: ev.clientX, y: ev.clientY }
  canvasEl.value.setPointerCapture(ev.pointerId)
}
function onPointerMove(ev) {
  if (!dragging) return
  const dx = ev.clientX - last.x
  const dy = ev.clientY - last.y
  last = { x: ev.clientX, y: ev.clientY }
  view.yaw += dx * 0.01
  view.pitch = Math.max(-1.35, Math.min(1.35, view.pitch + dy * 0.01))
  render()
}
function onPointerUp() {
  dragging = false
}

onMounted(() => {
  render()
  if (canvasEl.value && typeof ResizeObserver !== 'undefined') {
    observer = new ResizeObserver(() => render())
    observer.observe(canvasEl.value)
  }
})

watch(state, () => render())
watch(view, () => render())

onBeforeUnmount(() => {
  if (observer) observer.disconnect()
  observer = null
})
</script>

<template>
  <section>
    <div class="panel-head">
      <h2>3D Point Cloud</h2>
      <span class="meta">{{ cloudMeta }}</span>
    </div>
    <div class="canvas-wrap">
      <canvas
        ref="canvasEl"
        class="cloud-canvas"
        @pointerdown="onPointerDown"
        @pointermove="onPointerMove"
        @pointerup="onPointerUp"
        @pointerleave="onPointerUp"
      ></canvas>
    </div>
    <div class="readout">
      <div class="metric"><div class="label">Topic</div><div class="value">{{ topic }}</div></div>
      <div class="metric"><div class="label">Points</div><div class="value">{{ pointsText }}</div></div>
      <div class="metric"><div class="label">Frame</div><div class="value">{{ frame }}</div></div>
      <div class="metric"><div class="label">Age</div><div class="value">{{ age }}</div></div>
    </div>
  </section>
</template>
