// 3D point-cloud rendering, ported verbatim from the legacy drawCloud().
import { fmt } from './format.js'

export function rotatePoint(p, yaw, pitch) {
  const cy = Math.cos(yaw)
  const sy = Math.sin(yaw)
  const cp = Math.cos(pitch)
  const sp = Math.sin(pitch)
  const x1 = p[0] * cy - p[1] * sy
  const y1 = p[0] * sy + p[1] * cy
  const z1 = p[2]
  const y2 = y1 * cp - z1 * sp
  const z2 = y1 * sp + z1 * cp
  return [x1, y2, z2]
}

function drawCloudAxes(ctx, cx, cy, scale, yaw, pitch) {
  const axes = [
    { p: [0.6, 0, 0], c: '#dc2626', t: 'X' },
    { p: [0, 0.6, 0], c: '#16a34a', t: 'Y' },
    { p: [0, 0, 0.6], c: '#2563eb', t: 'Z' },
  ]
  ctx.lineWidth = 2 * (window.devicePixelRatio || 1)
  axes.forEach(a => {
    const r = rotatePoint(a.p, yaw, pitch)
    const x = cx + r[0] * scale
    const y = cy - r[1] * scale
    ctx.strokeStyle = a.c
    ctx.fillStyle = a.c
    ctx.beginPath()
    ctx.moveTo(cx, cy)
    ctx.lineTo(x, y)
    ctx.stroke()
    ctx.fillText(a.t, x + 4, y - 4)
  })
}

export function resizeCloudCanvas(canvas) {
  const rect = canvas.getBoundingClientRect()
  const dpr = window.devicePixelRatio || 1
  const w = Math.max(320, Math.floor(rect.width * dpr))
  const h = Math.max(240, Math.floor(rect.height * dpr))
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w
    canvas.height = h
  }
}

export function drawCloud(canvas, state, view, metaRef) {
  resizeCloudCanvas(canvas)
  const ctx = canvas.getContext('2d')
  const w = canvas.width
  const h = canvas.height
  ctx.clearRect(0, 0, w, h)
  ctx.fillStyle = '#fbfcfe'
  ctx.fillRect(0, 0, w, h)
  const cloud = state?.point_cloud
  const cx = w / 2
  const cy = h / 2
  drawCloudAxes(ctx, cx, cy, Math.min(w, h) * 0.22, view.yaw, view.pitch)

  if (!cloud || !cloud.points || cloud.points.length === 0) {
    ctx.fillStyle = '#66758a'
    ctx.fillText('waiting for PointCloud2 data', 16, 24)
    if (metaRef) metaRef.value = 'waiting'
    return
  }

  const pts = cloud.points
  let maxAbs = 0.2
  pts.forEach(p => {
    maxAbs = Math.max(maxAbs, Math.abs(p[0]), Math.abs(p[1]), Math.abs(p[2]))
  })
  const scale = (Math.min(w, h) * 0.42) / maxAbs
  const projected = pts.map(p => ({ p, r: rotatePoint(p, view.yaw, view.pitch) })).sort((a, b) => a.r[2] - b.r[2])
  projected.forEach(item => {
    const x = cx + item.r[0] * scale
    const y = cy - item.r[1] * scale
    const zNorm = Math.max(0, Math.min(1, (item.r[2] / maxAbs + 1) / 2))
    const radius = Math.max(3, Math.min(8, 4 + zNorm * 4)) * (window.devicePixelRatio || 1)
    ctx.fillStyle = `rgb(${Math.round(40 + zNorm * 170)}, ${Math.round(110 + zNorm * 70)}, ${Math.round(230 - zNorm * 90)})`
    ctx.beginPath()
    ctx.arc(x, y, radius, 0, Math.PI * 2)
    ctx.fill()
  })

  if (metaRef) {
    metaRef.value = `${cloud.sampled_points}/${cloud.total_points} points, age ${fmt(state.freshness_s.point_cloud, 2, 's')}`
  }
}
