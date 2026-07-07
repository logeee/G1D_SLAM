// SLAM map rendering, ported verbatim from the legacy drawMap() and helpers.
// The occupancy grid bitmap is cached by map.seq (expensive to rebuild).
import { fmt } from './format.js'
import { mapToCanvas } from './geometry.js'

let cachedMapSeq = -1
let cachedMapImage = null
let currentMapGeom = null

export function getCurrentMapGeom() {
  return currentMapGeom
}

export function resetMapCache() {
  cachedMapSeq = -1
}

export function resizeMapCanvas(canvas) {
  const rect = canvas.getBoundingClientRect()
  const dpr = window.devicePixelRatio || 1
  const w = Math.max(320, Math.floor(rect.width * dpr))
  const h = Math.max(240, Math.floor(rect.height * dpr))
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w
    canvas.height = h
    cachedMapSeq = -1
  }
}

function drawMapPolyline(ctx, map, geom, points, color, width, dashed = false) {
  if (!points || points.length < 2) return
  const dpr = window.devicePixelRatio || 1
  ctx.save()
  ctx.strokeStyle = color
  ctx.lineWidth = width * dpr
  ctx.lineJoin = 'round'
  ctx.lineCap = 'round'
  if (dashed) ctx.setLineDash([10 * dpr, 6 * dpr])
  ctx.beginPath()
  points.forEach((p, i) => {
    const c = mapToCanvas(map, p.x, p.y, geom)
    if (i === 0) ctx.moveTo(c.x, c.y)
    else ctx.lineTo(c.x, c.y)
  })
  ctx.stroke()
  ctx.restore()
}

function drawArrow(ctx, start, end, color, label) {
  const dpr = window.devicePixelRatio || 1
  const angle = Math.atan2(end.y - start.y, end.x - start.x)
  const head = 12 * dpr
  ctx.save()
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 4 * dpr
  ctx.lineCap = 'round'
  ctx.beginPath()
  ctx.moveTo(start.x, start.y)
  ctx.lineTo(end.x, end.y)
  ctx.stroke()
  ctx.beginPath()
  ctx.moveTo(end.x, end.y)
  ctx.lineTo(end.x - head * Math.cos(angle - Math.PI / 6), end.y - head * Math.sin(angle - Math.PI / 6))
  ctx.lineTo(end.x - head * Math.cos(angle + Math.PI / 6), end.y - head * Math.sin(angle + Math.PI / 6))
  ctx.closePath()
  ctx.fill()
  if (label) {
    ctx.font = `${12 * dpr}px system-ui, sans-serif`
    ctx.lineWidth = 4 * dpr
    ctx.strokeStyle = '#ffffff'
    ctx.strokeText(label, end.x + 8 * dpr, end.y - 8 * dpr)
    ctx.fillStyle = color
    ctx.fillText(label, end.x + 8 * dpr, end.y - 8 * dpr)
  }
  ctx.restore()
}

function drawTargetHeading(ctx, map, geom, nav) {
  const target = nav.targetYaw
  const waypoints = nav.selectedWaypoints
  if (!target || !waypoints.length) return
  const lastPoint = waypoints[waypoints.length - 1]
  const start = mapToCanvas(map, lastPoint.x, lastPoint.y, geom)
  let end
  if (nav.finalHeadingPoint && nav.manualHeadingDeg === null) {
    end = mapToCanvas(map, nav.finalHeadingPoint.x, nav.finalHeadingPoint.y, geom)
  } else {
    const dpr = window.devicePixelRatio || 1
    const len = 56 * dpr
    end = { x: start.x + Math.cos(target.yaw) * len, y: start.y - Math.sin(target.yaw) * len }
  }
  drawArrow(ctx, start, end, '#16a34a', `${target.label} ${target.yawDeg.toFixed(1)}°`)
}

function drawSelectedWaypoints(ctx, map, geom, nav) {
  const waypoints = nav.selectedWaypoints
  drawMapPolyline(ctx, map, geom, waypoints, '#9333ea', 3, true)
  const dpr = window.devicePixelRatio || 1
  waypoints.forEach((p, idx) => {
    const c = mapToCanvas(map, p.x, p.y, geom)
    ctx.fillStyle = '#9333ea'
    ctx.strokeStyle = '#ffffff'
    ctx.lineWidth = 3 * dpr
    ctx.beginPath()
    ctx.arc(c.x, c.y, 8 * dpr, 0, Math.PI * 2)
    ctx.fill()
    ctx.stroke()
    ctx.fillStyle = '#ffffff'
    ctx.font = `${12 * dpr}px system-ui, sans-serif`
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.fillText(String(idx + 1), c.x, c.y)
  })
  ctx.textAlign = 'start'
  ctx.textBaseline = 'alphabetic'
}

function drawSavedPoints(ctx, map, geom, nav) {
  const savedPoints = nav.savedPoints
  if (!savedPoints.length) return
  const dpr = window.devicePixelRatio || 1
  savedPoints.forEach(point => {
    if (point.x === null || point.x === undefined || point.y === null || point.y === undefined) return
    const c = mapToCanvas(map, point.x, point.y, geom)
    const active = point.id === nav.editingPointId
    ctx.save()
    ctx.fillStyle = active ? '#f59e0b' : '#0ea5e9'
    ctx.strokeStyle = '#ffffff'
    ctx.lineWidth = 3 * dpr
    ctx.beginPath()
    ctx.arc(c.x, c.y, (active ? 9 : 7) * dpr, 0, Math.PI * 2)
    ctx.fill()
    ctx.stroke()
    if (point.yaw !== null && point.yaw !== undefined) {
      const len = 28 * dpr
      drawArrow(ctx, c, { x: c.x + Math.cos(point.yaw) * len, y: c.y - Math.sin(point.yaw) * len }, '#0f766e', '')
    }
    ctx.font = `${11 * dpr}px system-ui, sans-serif`
    ctx.lineWidth = 3 * dpr
    ctx.strokeStyle = '#ffffff'
    ctx.fillStyle = '#0f172a'
    const label = point.name || 'Point'
    ctx.strokeText(label, c.x + 10 * dpr, c.y - 10 * dpr)
    ctx.fillText(label, c.x + 10 * dpr, c.y - 10 * dpr)
    ctx.restore()
  })
}

function drawGlobalPlan(ctx, map, geom, navState) {
  const path = navState?.global_plan_path?.poses || []
  drawMapPolyline(ctx, map, geom, path, '#d97706', 4, false)
}

// nav: { selectedWaypoints, savedPoints, editingPointId, finalHeadingPoint, manualHeadingDeg, targetYaw }
export function drawMap(canvas, state, nav, metaRef) {
  resizeMapCanvas(canvas)
  const ctx = canvas.getContext('2d')
  ctx.clearRect(0, 0, canvas.width, canvas.height)
  ctx.fillStyle = '#f8fafc'
  ctx.fillRect(0, 0, canvas.width, canvas.height)
  const map = state?.map
  if (!map || !map.data) {
    ctx.fillStyle = '#66758a'
    ctx.fillText('waiting for map', 16, 24)
    if (metaRef) metaRef.value = 'waiting'
    return
  }

  if (cachedMapSeq !== map.seq) {
    const off = document.createElement('canvas')
    off.width = map.width
    off.height = map.height
    const offCtx = off.getContext('2d')
    const img = offCtx.createImageData(map.width, map.height)
    for (let y = 0; y < map.height; y++) {
      for (let x = 0; x < map.width; x++) {
        const srcIdx = y * map.width + x
        const dstY = map.height - 1 - y
        const dstIdx = (dstY * map.width + x) * 4
        const v = map.data[srcIdx]
        let r = 224
        let g = 229
        let b = 236
        if (v === 0) {
          r = 255
          g = 255
          b = 255
        } else if (v > 70) {
          r = 28
          g = 38
          b = 52
        } else if (v > 0) {
          r = 120
          g = 132
          b = 150
        }
        img.data[dstIdx] = r
        img.data[dstIdx + 1] = g
        img.data[dstIdx + 2] = b
        img.data[dstIdx + 3] = 255
      }
    }
    offCtx.putImageData(img, 0, 0)
    cachedMapImage = off
    cachedMapSeq = map.seq
  }

  const scale = Math.min(canvas.width / map.width, canvas.height / map.height)
  const dw = map.width * scale
  const dh = map.height * scale
  const geom = { scale, ox: (canvas.width - dw) / 2, oy: (canvas.height - dh) / 2 }
  currentMapGeom = geom
  ctx.drawImage(cachedMapImage, geom.ox, geom.oy, dw, dh)

  drawGlobalPlan(ctx, map, geom, state.navigation)

  if (state.track && state.track.length > 1) {
    ctx.strokeStyle = '#2563eb'
    ctx.lineWidth = Math.max(2, 2 * (window.devicePixelRatio || 1))
    ctx.beginPath()
    state.track.forEach((p, i) => {
      const c = mapToCanvas(map, p.x, p.y, geom)
      if (i === 0) ctx.moveTo(c.x, c.y)
      else ctx.lineTo(c.x, c.y)
    })
    ctx.stroke()
  }

  if (state.odom) {
    const c = mapToCanvas(map, state.odom.x, state.odom.y, geom)
    const yaw = state.odom.yaw || 0
    const size = 12 * (window.devicePixelRatio || 1)
    ctx.save()
    ctx.translate(c.x, c.y)
    ctx.rotate(-yaw)
    ctx.fillStyle = '#dc2626'
    ctx.beginPath()
    ctx.moveTo(size, 0)
    ctx.lineTo(-size * 0.75, size * 0.6)
    ctx.lineTo(-size * 0.45, 0)
    ctx.lineTo(-size * 0.75, -size * 0.6)
    ctx.closePath()
    ctx.fill()
    ctx.restore()
  }

  drawSavedPoints(ctx, map, geom, nav)
  drawSelectedWaypoints(ctx, map, geom, nav)
  drawTargetHeading(ctx, map, geom, nav)

  if (metaRef) metaRef.value = `${map.width}x${map.height}, ${fmt(map.resolution, 3, 'm/cell')}`
}
