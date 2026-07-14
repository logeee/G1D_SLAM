import { onMounted, onBeforeUnmount, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'

// Ported from the legacy resizeCanvas(): keep the backing store sized to the
// element * devicePixelRatio for crisp rendering.
export function resizeCanvas(canvas) {
  const rect = canvas.getBoundingClientRect()
  const dpr = window.devicePixelRatio || 1
  const w = Math.max(320, Math.floor(rect.width * dpr))
  const h = Math.max(240, Math.floor(rect.height * dpr))
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w
    canvas.height = h
  }
}

// Binds an imperative canvas draw function to the telemetry store. The canvas
// stays fully imperative (no reactivity in the hot draw path); it just redraws
// whenever a fresh state snapshot arrives or the element is resized.
export function useCanvasRenderer(canvasRef, draw) {
  const store = useTelemetryStore()
  const { state } = storeToRefs(store)
  let observer = null

  function render() {
    const canvas = canvasRef.value
    if (!canvas) return
    resizeCanvas(canvas)
    draw(canvas, state.value)
  }

  onMounted(() => {
    render()
    if (canvasRef.value && typeof ResizeObserver !== 'undefined') {
      observer = new ResizeObserver(() => render())
      observer.observe(canvasRef.value)
    }
  })

  watch(state, () => render())

  onBeforeUnmount(() => {
    if (observer) observer.disconnect()
    observer = null
  })

  return { render }
}
