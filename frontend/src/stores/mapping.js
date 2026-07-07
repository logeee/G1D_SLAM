import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'
import { resetMapCache } from '../utils/mapDraw.js'
import { useDialogStore } from './dialog.js'

// View switching (dashboard/mapping) + mapping session controls.
export const useMappingStore = defineStore('mapping', () => {
  const activeView = ref('dashboard') // 'dashboard' | 'mapping'
  const busy = ref(false)
  // 「准备采集」:先显示右侧相机+遥控,让操作者把机器人开到想开始的位置,
  // 再点「确认开始采集」真正清图建图。prepared 只是前端 UI 状态,不动底盘。
  const prepared = ref(false)

  function enterMapping() {
    activeView.value = 'mapping'
    resetMapCache()
  }

  function exitMapping() {
    activeView.value = 'dashboard'
    prepared.value = false
    resetMapCache()
  }

  function prepareMapping() {
    prepared.value = true
  }

  function cancelPrepare() {
    prepared.value = false
  }

  async function startMapping() {
    const dialog = useDialogStore()
    const ok = await dialog.confirm(
      '开始重新建图？此操作会清空底盘上当前的地图。\n（备注：开始采集只清空底盘当前地图，已存档的地图文件不会被清空。）\n\nStart a fresh map? This will CLEAR the current map on the chassis.\n(Note: starting a new session only clears the chassis map; archived map files are kept.)',
      { title: '开始建图 / Start Mapping', confirmText: '开始建图', danger: true },
    )
    if (!ok) return
    busy.value = true
    try {
      const data = await api.post('/api/mapping/start', { clear: true })
      if (!data.ok) await dialog.alert('开始采集失败 / Start mapping failed: ' + (data.error || 'unknown'), { title: '出错' })
    } catch (err) {
      await dialog.alert('开始采集出错 / Start mapping error: ' + err, { title: '出错' })
    } finally {
      busy.value = false
    }
  }

  async function stopMapping() {
    const dialog = useDialogStore()
    busy.value = true
    try {
      const data = await api.post('/api/mapping/stop', {})
      if (!data.ok) await dialog.alert('结束采集失败 / Stop mapping failed: ' + (data.error || 'unknown'), { title: '出错' })
      else prepared.value = false
    } catch (err) {
      await dialog.alert('结束采集出错 / Stop mapping error: ' + err, { title: '出错' })
    } finally {
      busy.value = false
    }
  }

  async function saveMap() {
    const dialog = useDialogStore()
    const name = await dialog.prompt('输入地图名称（例如 八维通 或 八维通.stcm）\nEnter map name (e.g. Baweitong or Baweitong.stcm):', '', {
      title: '保存地图 / Save Map',
      placeholder: '八维通',
      confirmText: '保存',
    })
    if (!name || !String(name).trim()) return
    busy.value = true
    try {
      const data = await api.post('/api/mapping/save', { name: String(name).trim() })
      if (!data.ok) {
        await dialog.alert('保存地图失败 / Save map failed: ' + (data.error || 'unknown'), { title: '出错' })
        return
      }
      await dialog.alert(`地图已保存 / Map saved:\n${data.path}\n${data.size_bytes} bytes`, { title: '已保存 / Saved' })
    } catch (err) {
      await dialog.alert('保存地图出错 / Save map error: ' + err, { title: '出错' })
    } finally {
      busy.value = false
    }
  }

  async function loadMap() {
    const dialog = useDialogStore()
    let maps = []
    try {
      const listData = await api.getMappingList()
      maps = listData.saved_maps || []
    } catch (err) {
      await dialog.alert('读取地图列表失败 / Failed to list maps: ' + err, { title: '出错' })
      return
    }
    if (!maps.length) {
      await dialog.alert('暂无已存档地图，请先保存地图。\n\nNo archived maps found. Save a map first.', { title: '加载地图 / Load Map' })
      return
    }

    const fmtSize = bytes => (bytes >= 1024 * 1024 ? `${(bytes / 1024 / 1024).toFixed(1)} MB` : `${(bytes / 1024).toFixed(0)} KB`)
    const items = maps.map(m => ({ value: m.name, label: `${m.name} (${fmtSize(m.size_bytes)})` }))
    const defaultValue = maps[maps.length - 1].name

    const name = await dialog.select('选择要加载的地图 / Choose a map to load:', items, {
      title: '加载地图 / Load Map',
      defaultValue,
      confirmText: '下一步',
    })
    if (!name || !String(name).trim()) return

    const ok = await dialog.confirm(
      `将把「${name}」加载到底盘，覆盖底盘当前地图。\n\nLoad "${name}" into the chassis and replace the current chassis map?`,
      { title: '确认加载 / Confirm Load', confirmText: '加载', danger: true },
    )
    if (!ok) return
    busy.value = true
    try {
      const data = await api.post('/api/mapping/load', { name: String(name).trim() })
      if (!data.ok) {
        await dialog.alert('加载地图失败 / Load map failed: ' + (data.error || 'unknown'), { title: '出错' })
        return
      }
      await dialog.alert(`地图已加载 / Map loaded:\n${data.path}\n${data.size_bytes} bytes`, { title: '已加载 / Loaded' })
      resetMapCache()
    } catch (err) {
      await dialog.alert('加载地图出错 / Load map error: ' + err, { title: '出错' })
    } finally {
      busy.value = false
    }
  }

  return { activeView, busy, prepared, enterMapping, exitMapping, prepareMapping, cancelPrepare, startMapping, stopMapping, saveMap, loadMap }
})
