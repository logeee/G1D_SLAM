import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'

// Saved point library (/api/points). Holds the list + which point is being
// edited (also used to highlight it on the map). Form fields live in the
// PointsPanel component; this store owns data + persistence.
export const usePointsStore = defineStore('points', () => {
  const savedPoints = ref([])
  const editingPointId = ref(null)
  const loadError = ref('')

  function getById(pointId) {
    return savedPoints.value.find(point => point.id === pointId) || null
  }

  async function load() {
    const data = await api.getPoints()
    if (!data.ok) throw new Error(data.error || 'load points failed')
    savedPoints.value = data.points || []
    if (editingPointId.value && !savedPoints.value.some(p => p.id === editingPointId.value)) {
      editingPointId.value = null
    }
    loadError.value = ''
    return savedPoints.value
  }

  async function recordCurrent(payload) {
    return api.post('/api/points/record_current', payload)
  }

  async function upsert(payload) {
    return api.post('/api/points/upsert', payload)
  }

  async function remove(id) {
    return api.post('/api/points/delete', { id })
  }

  return { savedPoints, editingPointId, loadError, getById, load, recordCurrent, upsert, remove }
})
