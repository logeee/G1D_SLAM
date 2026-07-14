import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { api } from '../api/client.js'

// 2D 重定位流程(Mapping Mode):三种方式 + 结果 + 应用到底盘。
//   global : 雷达全局匹配(无初值)
//   json   : 双阶段 ICP,初值 = 定时保存的上次位姿
//   click  : 双阶段 ICP,初值 = 地图上手动点击
export const useReloc2dStore = defineStore('reloc2d', () => {
  const open = ref(false) // 弹窗可见
  const step = ref('choose') // 'choose' | 'result'
  const running = ref(false) // 正在跑匹配
  const picking = ref(false) // 等待地图点击选点(此时弹窗暂时隐藏)
  const result = ref(null) // 匹配结果
  const error = ref(null)
  const applied = ref(false) // 已应用到底盘
  const applyError = ref(null)

  // 定时保存位姿的配置(运行时可调)
  const saveIntervalSec = ref(10) // 输入框绑定值
  const config = ref(null) // { interval_sec, enabled, last_pose, path }
  const configBusy = ref(false)

  // 候选位姿:结果出来后在地图上预览(橙色箭头),让用户看清落点再决定是否应用。
  const previewPose = computed(() => {
    // 窗口关闭(以任何方式)后不再显示候选箭头。
    if (!open.value || step.value !== 'result' || running.value || error.value) return null
    const p = result.value && result.value.pose
    if (!p || p.x === null || p.x === undefined || p.y === null || p.y === undefined) return null
    return { x: p.x, y: p.y, yaw: p.yaw, accepted: !!result.value.accepted }
  })

  async function loadConfig() {
    try {
      const data = await api.getReloc2dConfig()
      config.value = data
      if (data && data.interval_sec !== undefined && data.interval_sec !== null) {
        saveIntervalSec.value = data.interval_sec
      }
    } catch (err) {
      // 读不到就用默认值,不打断主流程
    }
  }

  async function applyInterval() {
    const sec = Number(saveIntervalSec.value)
    if (!Number.isFinite(sec)) return
    configBusy.value = true
    try {
      const data = await api.post('/api/reloc2d/config', { interval_sec: sec })
      if (data && data.ok) {
        config.value = data
        if (data.interval_sec !== undefined) saveIntervalSec.value = data.interval_sec
      }
    } catch (err) {
      // 忽略,保持原值
    } finally {
      configBusy.value = false
    }
  }

  function openDialog() {
    open.value = true
    step.value = 'choose'
    result.value = null
    error.value = null
    picking.value = false
    loadConfig()
  }

  function close() {
    open.value = false
    picking.value = false
  }

  async function run(method, init) {
    running.value = true
    error.value = null
    applied.value = false
    applyError.value = null
    try {
      const data = await api.post('/api/reloc2d/run', { method, init, apply: false })
      if (!data.ok) {
        error.value = data.error || '匹配失败'
        result.value = null
      } else {
        result.value = data
      }
      step.value = 'result'
    } catch (err) {
      error.value = String(err)
      step.value = 'result'
    } finally {
      running.value = false
    }
  }

  function runGlobal() {
    return run('global', null)
  }

  function runJson() {
    return run('json', null)
  }

  // 手动选点:隐藏弹窗、进入地图拾取;由 MappingView 的 @pick 回调 onPicked。
  function beginClickPick() {
    picking.value = true
    open.value = false
  }

  function onPicked(point) {
    picking.value = false
    open.value = true
    if (!point || point.x === undefined || point.x === null) {
      error.value = '点在地图外,请重新选点'
      step.value = 'result'
      return
    }
    run('click', { x: point.x, y: point.y })
  }

  function cancelPick() {
    picking.value = false
    open.value = true
    step.value = 'choose'
  }

  // 把当前结果的位姿喂给底盘(set_pose + recover_localization),复用现有 anchor 通道。
  // 直接应用,不再二次确认;成功/失败就地内联显示(避免叠一层弹窗)。
  async function applyToChassis() {
    if (!result.value || !result.value.pose) return
    const p = result.value.pose
    running.value = true
    applyError.value = null
    try {
      const data = await api.post('/api/relocalization/run', {
        anchor: { x: p.x, y: p.y, yaw: p.yaw },
      })
      if (data.ok) {
        applied.value = true
      } else {
        applyError.value = data.error || 'unknown'
      }
    } catch (err) {
      applyError.value = String(err)
    } finally {
      running.value = false
    }
  }

  return {
    open, step, running, picking, result, error, applied, applyError, previewPose,
    saveIntervalSec, config, configBusy, loadConfig, applyInterval,
    openDialog, close, runGlobal, runJson, beginClickPick, onPicked, cancelPick, applyToChassis,
  }
})
