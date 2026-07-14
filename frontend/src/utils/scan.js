// Laser-scan analysis ported verbatim from the legacy embedded frontend.
import { normalizeDeg } from './format.js'

export const SCAN_SECTOR_DEFS = [
  { id: 'front', label: '\u524d\u65b9', from: -25, to: 25, warn: 0.8, danger: 0.45 },
  { id: 'left_front', label: '\u5de6\u524d', from: 25, to: 90, warn: 0.65, danger: 0.4 },
  { id: 'right_front', label: '\u53f3\u524d', from: -90, to: -25, warn: 0.65, danger: 0.4 },
  { id: 'left_rear', label: '\u5de6\u540e', from: 90, to: 160, warn: 0.45, danger: 0.3 },
  { id: 'right_rear', label: '\u53f3\u540e', from: -160, to: -90, warn: 0.45, danger: 0.3 },
  { id: 'rear', label: '\u540e\u65b9', from: 160, to: -160, wrap: true, warn: 0.45, danger: 0.3 },
]

export function angleInSector(deg, sector) {
  const v = normalizeDeg(deg)
  if (sector.wrap) return v >= sector.from || v <= sector.to
  return v >= sector.from && v <= sector.to
}

export function targetRelativeDeg(state) {
  const waypoint = state?.navigation?.last_command?.waypoints?.[0]
  const odom = state?.odom
  if (
    !waypoint ||
    !odom ||
    !Number.isFinite(Number(odom.x)) ||
    !Number.isFinite(Number(odom.y)) ||
    !Number.isFinite(Number(odom.yaw))
  )
    return null
  const heading = Math.atan2(Number(waypoint.y) - Number(odom.y), Number(waypoint.x) - Number(odom.x))
  return normalizeDeg((heading - Number(odom.yaw)) * 180 / Math.PI)
}

export function analyzeScan(state) {
  const scan = state?.scan
  const result = {
    ok: false,
    status: 'waiting',
    label: '\u7b49\u5f85\u6fc0\u5149\u6570\u636e',
    minRange: null,
    minAngleDeg: null,
    sectors: [],
    target: null,
    dangerCount: 0,
    warnCount: 0,
  }
  if (!scan || !Array.isArray(scan.ranges)) return result
  const sectors = SCAN_SECTOR_DEFS.map(def => ({
    ...def,
    minRange: null,
    minAngleDeg: null,
    point: null,
    count: 0,
    status: 'ok',
  }))
  const targetDeg = targetRelativeDeg(state)
  const target =
    targetDeg === null
      ? null
      : {
          label: '\u76ee\u6807\u65b9\u5411',
          centerDeg: targetDeg,
          halfDeg: 10,
          minRange: null,
          minAngleDeg: null,
          point: null,
          count: 0,
          status: 'ok',
          warn: 0.8,
          danger: 0.45,
        }
  for (let i = 0; i < scan.ranges.length; i++) {
    const r = Number(scan.ranges[i])
    if (!Number.isFinite(r) || r <= 0) continue
    const angle = Number(scan.angle_min) + i * Number(scan.angle_increment)
    const deg = normalizeDeg((angle * 180) / Math.PI)
    const point = { x: r * Math.cos(angle), y: r * Math.sin(angle), angle, deg, range: r }
    if (result.minRange === null || r < result.minRange) {
      result.minRange = r
      result.minAngleDeg = deg
    }
    for (const sector of sectors) {
      if (!angleInSector(deg, sector)) continue
      sector.count += 1
      if (sector.minRange === null || r < sector.minRange) {
        sector.minRange = r
        sector.minAngleDeg = deg
        sector.point = point
      }
    }
    if (target) {
      const diff = Math.abs(normalizeDeg(deg - target.centerDeg))
      if (diff <= target.halfDeg) {
        target.count += 1
        if (target.minRange === null || r < target.minRange) {
          target.minRange = r
          target.minAngleDeg = deg
          target.point = point
        }
      }
    }
  }
  for (const sector of sectors) {
    if (sector.minRange !== null && sector.minRange <= sector.danger) {
      sector.status = 'danger'
      result.dangerCount += 1
    } else if (sector.minRange !== null && sector.minRange <= sector.warn) {
      sector.status = 'warn'
      result.warnCount += 1
    }
  }
  if (target) {
    if (target.minRange !== null && target.minRange <= target.danger) target.status = 'danger'
    else if (target.minRange !== null && target.minRange <= target.warn) target.status = 'warn'
  }
  result.ok = true
  result.sectors = sectors
  result.target = target
  if (result.dangerCount > 0 || target?.status === 'danger') {
    result.status = 'danger'
    result.label = '\u8fd1\u969c\u788d\u5371\u9669'
  } else if (result.warnCount > 0 || target?.status === 'warn') {
    result.status = 'warn'
    result.label = '\u8fd1\u969c\u788d\u6ce8\u610f'
  } else {
    result.status = 'ok'
    result.label = '\u6fc0\u5149\u8fd1\u969c\u788d\u6b63\u5e38'
  }
  return result
}
