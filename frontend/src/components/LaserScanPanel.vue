<script setup>
import { computed, ref } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useCanvasRenderer } from '../composables/useCanvasRenderer.js'
import { analyzeScan } from '../utils/scan.js'
import { fmt } from '../utils/format.js'

const store = useTelemetryStore()
const { state } = storeToRefs(store)

const scanCanvas = ref(null)

// Reactive readouts / meta / alert derive from the snapshot.
const scan = computed(() => state.value?.scan || null)
const freshness = computed(() => state.value?.freshness_s || {})
const analysis = computed(() => analyzeScan(state.value || {}))

const meta = computed(() =>
  scan.value
    ? `${scan.value.count} rays, age ${fmt(freshness.value.scan, 2, 's')} / ${analysis.value.label}`
    : 'waiting',
)
const valid = computed(() => (scan.value ? `${scan.value.valid_count}/${scan.value.count}` : '--'))
const minRange = computed(() => fmt(scan.value?.min_range, 3, 'm'))
const frame = computed(() => scan.value?.frame_id || '--')
const age = computed(() => fmt(freshness.value.scan, 2, 's'))

function sectorStatusText(status) {
  return status === 'danger' ? '\u5371\u9669' : status === 'warn' ? '\u6ce8\u610f' : 'OK'
}

// Ported verbatim from the legacy drawScan().
function drawScan(canvas, snapshot) {
  const ctx = canvas.getContext('2d')
  const w = canvas.width
  const h = canvas.height
  ctx.clearRect(0, 0, w, h)
  ctx.fillStyle = '#fbfcfe'
  ctx.fillRect(0, 0, w, h)
  const s = snapshot?.scan
  const ana = analyzeScan(snapshot || {})
  const cx = w / 2
  const cy = h * 0.58
  const maxM = s?.range_max ? Math.min(s.range_max, 8) : 8
  const scale = (Math.min(w, h) * 0.42) / maxM

  ctx.strokeStyle = '#d8dee8'
  ctx.lineWidth = 1
  for (let m = 1; m <= maxM; m++) {
    ctx.beginPath()
    ctx.arc(cx, cy, m * scale, 0, Math.PI * 2)
    ctx.stroke()
  }
  ctx.strokeStyle = '#94a3b8'
  ctx.beginPath()
  ctx.moveTo(cx, cy)
  ctx.lineTo(cx, cy - maxM * scale)
  ctx.stroke()

  ctx.setLineDash([8, 5])
  ctx.lineWidth = 2
  ctx.strokeStyle = '#facc15'
  ctx.beginPath()
  ctx.arc(cx, cy, 0.8 * scale, 0, Math.PI * 2)
  ctx.stroke()
  ctx.strokeStyle = '#ef4444'
  ctx.beginPath()
  ctx.arc(cx, cy, 0.45 * scale, 0, Math.PI * 2)
  ctx.stroke()
  ctx.setLineDash([])

  ctx.fillStyle = '#2563eb'
  if (s && s.ranges) {
    for (let i = 0; i < s.ranges.length; i++) {
      const r = s.ranges[i]
      if (r === null) continue
      const a = s.angle_min + i * s.angle_increment
      const x = r * Math.cos(a)
      const y = r * Math.sin(a)
      const px = cx - y * scale
      const py = cy - x * scale
      ctx.fillRect(px - 1.5, py - 1.5, 3, 3)
    }
    if (ana.ok) {
      for (const sector of ana.sectors) {
        if (!sector.point || sector.status === 'ok') continue
        const px = cx - sector.point.y * scale
        const py = cy - sector.point.x * scale
        ctx.fillStyle = sector.status === 'danger' ? '#ef4444' : '#f59e0b'
        ctx.beginPath()
        ctx.arc(px, py, sector.status === 'danger' ? 7 : 5, 0, Math.PI * 2)
        ctx.fill()
        ctx.fillStyle = '#111827'
        ctx.font = '12px system-ui'
        ctx.fillText(`${sector.label} ${sector.minRange.toFixed(2)}m`, px + 8, py - 8)
      }
      if (ana.target) {
        const targetRad = (ana.target.centerDeg * Math.PI) / 180
        ctx.strokeStyle = '#16a34a'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(cx, cy)
        ctx.lineTo(cx - Math.sin(targetRad) * maxM * scale * 0.75, cy - Math.cos(targetRad) * maxM * scale * 0.75)
        ctx.stroke()
        ctx.fillStyle = '#16a34a'
        ctx.font = '12px system-ui'
        ctx.fillText(`\u76ee\u6807 ${ana.target.centerDeg.toFixed(0)}\u00b0`, 12, h - 16)
      }
    }
  } else {
    ctx.fillStyle = '#66758a'
    ctx.fillText('waiting for laser scan', 16, 24)
  }
  ctx.fillStyle = '#991b1b'
  ctx.font = '12px system-ui'
  ctx.fillText('\u7ea2\u5708<=0.45m  \u9ec4\u5708<=0.80m', 12, 20)
}

useCanvasRenderer(scanCanvas, drawScan)
</script>

<template>
  <section>
    <div class="panel-head">
      <h2>Laser Scan</h2>
      <span class="meta">{{ meta }}</span>
    </div>
    <div class="canvas-wrap"><canvas ref="scanCanvas" class="scan-canvas"></canvas></div>
    <div class="readout">
      <div class="metric"><div class="label">Valid</div><div class="value">{{ valid }}</div></div>
      <div class="metric"><div class="label">Min Range</div><div class="value">{{ minRange }}</div></div>
      <div class="metric"><div class="label">Frame</div><div class="value">{{ frame }}</div></div>
      <div class="metric"><div class="label">Age</div><div class="value">{{ age }}</div></div>
    </div>
    <div class="scan-alert">
      <template v-if="!analysis.ok">
        <div class="scan-alert-summary">等待 Laser Scan 数据</div>
      </template>
      <template v-else>
        <div class="scan-alert-summary" :class="analysis.status">
          <strong>{{ analysis.label }}</strong
          >：这是页面基于 Laser Scan 的诊断阈值，不等同于 Slamware 内部报警。
          <template v-if="analysis.target">
            目标方向 {{ analysis.target.centerDeg.toFixed(0) }}°，±{{ analysis.target.halfDeg }}° 内最近
            {{ analysis.target.minRange === null ? '--' : analysis.target.minRange.toFixed(2) + 'm' }}
          </template>
          <template v-else>没有正在执行的目标方向</template>
        </div>
        <div class="scan-sector-grid">
          <div v-for="sector in analysis.sectors" :key="sector.id" class="scan-sector-card" :class="sector.status">
            <div class="scan-sector-title">
              <span>{{ sector.label }}</span><span>{{ sectorStatusText(sector.status) }}</span>
            </div>
            <div class="scan-sector-meta">
              最近 {{ sector.minRange === null ? '--' : sector.minRange.toFixed(2) + ' m' }} /
              {{ sector.minAngleDeg === null ? '--' : sector.minAngleDeg.toFixed(0) + '°' }}
            </div>
            <div class="scan-sector-meta">
              黄&lt;={{ sector.warn.toFixed(2) }}m　红&lt;={{ sector.danger.toFixed(2) }}m
            </div>
          </div>
        </div>
      </template>
    </div>
  </section>
</template>
