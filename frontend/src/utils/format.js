// Ported verbatim from the legacy embedded frontend (fmt / angle helpers).

export function fmt(value, digits = 2, suffix = '') {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return '--'
  return `${Number(value).toFixed(digits)}${suffix}`
}

export function normalizeAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle))
}

export function radToDeg(angle) {
  return (normalizeAngle(angle) * 180) / Math.PI
}

export function normalizeDeg(deg) {
  let v = Number(deg)
  while (v > 180) v -= 360
  while (v <= -180) v += 360
  return v
}

export function clampSpeedRatio(value, fallback = 1) {
  const raw = Number(value)
  const base = Number.isFinite(raw) ? raw : Number(fallback)
  return Math.max(0.05, Math.min(1.0, Number.isFinite(base) ? base : 1))
}

export function angleDiffDeg(aDeg, bDeg) {
  if (!Number.isFinite(Number(aDeg)) || !Number.isFinite(Number(bDeg))) return null
  return Math.abs((normalizeAngle((Number(aDeg) - Number(bDeg)) * Math.PI / 180) * 180) / Math.PI)
}
