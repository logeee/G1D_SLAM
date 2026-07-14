<script setup>
import { onMounted, onBeforeUnmount, ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useWorkflowStore } from '../stores/workflow.js'
import { usePointsStore } from '../stores/points.js'
import { useReloc2dStore } from '../stores/reloc2d.js'
import { drawMap, getCurrentMapGeom } from '../utils/mapDraw.js'
import { canvasToMap, eventToCanvas } from '../utils/geometry.js'

const props = defineProps({
  interactive: { type: Boolean, default: false },
  // Pick mode: clicking reports the map-frame {x,y} via @pick instead of adding a waypoint.
  pickMode: { type: Boolean, default: false },
  // Whether to render saved nav points (hidden in Mapping Mode for a cleaner view).
  showSavedPoints: { type: Boolean, default: true },
})
const emit = defineEmits(['pick'])

const telemetry = useTelemetryStore()
const workflow = useWorkflowStore()
const points = usePointsStore()
const reloc = useReloc2dStore()
const { state } = storeToRefs(telemetry)
const { selectedWaypoints, finalHeadingPoint, manualHeadingDeg } = storeToRefs(workflow)
const { savedPoints, editingPointId } = storeToRefs(points)
const { previewPose } = storeToRefs(reloc)

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
    showSavedPoints: props.showSavedPoints,
    previewPose: previewPose.value,
  }
  drawMap(canvas, state.value, nav)
}

function onClick(ev) {
  if (!props.interactive && !props.pickMode) return
  const st = state.value
  const geom = getCurrentMapGeom()
  if (!st?.map || !geom) return
  const c = eventToCanvas(canvasEl.value, ev)
  const p = canvasToMap(st.map, c.x, c.y, geom)
  if (!p) {
    if (props.pickMode) emit('pick', null)
    else workflow.showNavMessage('bad', '导航：<strong>点在地图外</strong>')
    return
  }
  if (props.pickMode) {
    emit('pick', { x: p.x, y: p.y })
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
  [state, selectedWaypoints, savedPoints, editingPointId, finalHeadingPoint, manualHeadingDeg, previewPose],
  () => render(),
  { deep: false },
)

onBeforeUnmount(() => {
  if (observer) observer.disconnect()
  observer = null
})
</script>

<template>
  <!-- When not interactive (e.g. Mapping Mode) clicks are a no-op; show the
       default cursor instead of the crosshair so it doesn't look clickable. -->
  <canvas
    ref="canvasEl"
    class="map-canvas"
    :style="(interactive || pickMode) ? { cursor: 'crosshair' } : { cursor: 'default' }"
    @click="onClick"
  ></canvas>
</template>
