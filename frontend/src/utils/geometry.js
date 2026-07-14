// Map <-> canvas coordinate helpers, ported verbatim from the legacy frontend.

export function mapToCanvas(map, x, y, geom) {
  const mx = (x - map.origin.x) / map.resolution
  const my = (y - map.origin.y) / map.resolution
  return {
    x: geom.ox + mx * geom.scale,
    y: geom.oy + (map.height - my) * geom.scale,
  }
}

export function canvasToMap(map, canvasX, canvasY, geom) {
  if (!map || !geom) return null
  const mx = (canvasX - geom.ox) / geom.scale
  const my = map.height - (canvasY - geom.oy) / geom.scale
  if (mx < 0 || my < 0 || mx >= map.width || my >= map.height) return null
  return {
    x: map.origin.x + mx * map.resolution,
    y: map.origin.y + my * map.resolution,
  }
}

export function eventToCanvas(canvas, ev) {
  const rect = canvas.getBoundingClientRect()
  return {
    x: (ev.clientX - rect.left) * (canvas.width / rect.width),
    y: (ev.clientY - rect.top) * (canvas.height / rect.height),
  }
}
