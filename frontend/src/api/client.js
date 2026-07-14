// Thin wrappers around the backend REST API. Every path is relative so it works
// both behind the Vite dev proxy and when served from the backend's dist/.

async function getJson(url) {
  const res = await fetch(url, { cache: 'no-store' })
  if (!res.ok) throw new Error(`GET ${url} -> ${res.status}`)
  return res.json()
}

async function postJson(url, payload = {}) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  })
  const data = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(data?.error || `POST ${url} -> ${res.status}`)
  return data
}

export const api = {
  getState: () => getJson('/api/state'),
  getPoints: () => getJson('/api/points'),
  getWorkflows: () => getJson('/api/workflows'),
  getLiftHeight: () => getJson('/api/lift_height'),
  getFaultSnapshots: () => getJson('/api/fault_snapshots'),
  getRelocalizationStatus: () => getJson('/api/relocalization/status'),
  getMappingStatus: () => getJson('/api/mapping/status'),
  getMappingList: () => getJson('/api/mapping/list'),
  getReloc2dConfig: () => getJson('/api/reloc2d/config'),
  getControlStatus: () => getJson('/api/control/status'),
  getHealth: () => getJson('/api/health'),
  post: postJson,
}
