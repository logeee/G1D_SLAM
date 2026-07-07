import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'
import { resetMapCache } from '../utils/mapDraw.js'

// View switching (dashboard/mapping) + mapping session controls.
export const useMappingStore = defineStore('mapping', () => {
  const activeView = ref('dashboard') // 'dashboard' | 'mapping'
  const busy = ref(false)

  function enterMapping() {
    activeView.value = 'mapping'
    resetMapCache()
  }

  function exitMapping() {
    activeView.value = 'dashboard'
    resetMapCache()
  }

  async function startMapping() {
    if (
      !confirm(
        '开始重新建图？此操作会清空底盘上当前的地图。\n（备注：开始采集只清空底盘当前地图，已存档的地图文件不会被清空。）\nStart a fresh map? This will CLEAR the current map on the chassis.\n(Note: starting a new session only clears the chassis map; archived map files are kept.)',
      )
    )
      return
    busy.value = true
    try {
      const data = await api.post('/api/mapping/start', { clear: true })
      if (!data.ok) alert('开始采集失败 / Start mapping failed: ' + (data.error || 'unknown'))
    } catch (err) {
      alert('开始采集出错 / Start mapping error: ' + err)
    } finally {
      busy.value = false
    }
  }

  async function stopMapping() {
    busy.value = true
    try {
      const data = await api.post('/api/mapping/stop', {})
      if (!data.ok) alert('结束采集失败 / Stop mapping failed: ' + (data.error || 'unknown'))
    } catch (err) {
      alert('结束采集出错 / Stop mapping error: ' + err)
    } finally {
      busy.value = false
    }
  }

  async function saveMap() {
    const name = prompt('输入地图名称（例如 八维通 或 八维通.stcm）\nEnter map name (e.g. Baweitong or Baweitong.stcm):', '')
    if (!name || !String(name).trim()) return
    busy.value = true
    try {
      const data = await api.post('/api/mapping/save', { name: String(name).trim() })
      if (!data.ok) {
        alert('保存地图失败 / Save map failed: ' + (data.error || 'unknown'))
        return
      }
      alert(`地图已保存 / Map saved:\n${data.path}\n${data.size_bytes} bytes`)
    } catch (err) {
      alert('保存地图出错 / Save map error: ' + err)
    } finally {
      busy.value = false
    }
  }

  async function loadMap() {
    let hint = '输入要加载的地图名称 / Enter map name to load:'
    let defaultName = ''
    try {
      const listData = await api.getMappingList()
      const maps = listData.saved_maps || []
      if (maps.length) {
        defaultName = maps[maps.length - 1].name.replace(/\.stcm$/i, '')
        hint =
          '已存档地图 / Saved maps:\n' +
          maps.map(m => `- ${m.name} (${m.size_bytes} bytes)`).join('\n') +
          '\n\n输入要加载的名称 / Enter map name to load:'
      }
    } catch (err) {
      hint = '读取地图列表失败，仍可手动输入名称 / Failed to list maps, enter name manually:'
    }
    const name = prompt(hint, defaultName)
    if (!name || !String(name).trim()) return
    if (!confirm('将把已存档地图加载到底盘，覆盖底盘当前地图。\nLoad the archived map into the chassis and replace the current chassis map?')) return
    busy.value = true
    try {
      const data = await api.post('/api/mapping/load', { name: String(name).trim() })
      if (!data.ok) {
        alert('加载地图失败 / Load map failed: ' + (data.error || 'unknown'))
        return
      }
      alert(`地图已加载 / Map loaded:\n${data.path}\n${data.size_bytes} bytes`)
      resetMapCache()
    } catch (err) {
      alert('加载地图出错 / Load map error: ' + err)
    } finally {
      busy.value = false
    }
  }

  return { activeView, busy, enterMapping, exitMapping, startMapping, stopMapping, saveMap, loadMap }
})
