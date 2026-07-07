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
