<script setup>
import { onMounted, onBeforeUnmount, ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useWorkflowStore } from '../stores/workflow.js'
import { usePointsStore } from '../stores/points.js'
import { drawMap, getCurrentMapGeom } from '../utils/mapDraw.js'
import { canvasToMap, eventToCanvas } from '../utils/geometry.js'

const props = defineProps({
  interactive: { type: Boolean, default: false },
})

const telemetry = useTelemetryStore()
const workflow = useWorkflowStore()
const points = usePointsStore()
const { state } = storeToRefs(telemetry)
const { selectedWaypoints, finalHeadingPoint, manualHeadingDeg } = storeToRefs(workflow)
const { savedPoints, editingPointId } = storeToRefs(points)

const canvasEl = ref(null)
let observer = null

function render() {
  const canvas = canvasEl.value
  if (!canvas) return
  const nav = {
    selectedWaypoints: selectedWaypoints.value,
    savedPoints: savedPoints.value,
    editingPointId: editingPointId.value,
    finalHeadingPoint: finalHeadingPoint.value,
    manualHeadingDeg: manualHeadingDeg.value,
    targetYaw: workflow.computeTargetYaw(state.value),
  }
  drawMap(canvas, state.value, nav)
}

function onClick(ev) {
  if (!props.interactive) return
  const st = state.value
  const geom = getCurrentMapGeom()
  if (!st?.map || !geom) return
  const c = eventToCanvas(canvasEl.value, ev)
  const p = canvasToMap(st.map, c.x, c.y, geom)
  if (!p) {
    workflow.showNavMessage('bad', '导航：<strong>点在地图外</strong>')
    return
  }
  workflow.handleMapClick(p, st?.odom?.yaw_deg || 0)
}

onMounted(() => {
  render()
  if (canvasEl.value && typeof ResizeObserver !== 'undefined') {
    observer = new ResizeObserver(() => render())
    observer.observe(canvasEl.value)
  }
})

watch(
  [state, selectedWaypoints, savedPoints, editingPointId, finalHeadingPoint, manualHeadingDeg],
  () => render(),
  { deep: false },
)

onBeforeUnmount(() => {
  if (observer) observer.disconnect()
  observer = null
})
</script>

<template>
  <canvas ref="canvasEl" class="map-canvas" @click="onClick"></canvas>
</template>
