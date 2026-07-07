import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from '../api/client.js'

// 建图采集时的手动遥控(jog)。按住方向键持续动、松开即停。
//   - 按住: 每 HEARTBEAT_MS 向后端发一次 /api/control/jog(相同动作只刷新心跳,不重启进程)
//   - 松开/离开/切后台: 发 /api/control/stop
//   - 后端还有服务端「死人开关」看门狗,断连约 1s 内自动停,双保险
//
// 速度上限(滑条最大值)后端会在 /status 里回传;这里先给一份与后端一致的默认,
// 拿到 status 后覆盖。调好上限后改后端 --jog-max-* 或这里的默认即可。
const HEARTBEAT_MS = 300

export const useControlStore = defineStore('control', () => {
  // 滑条上限(可被后端 status 覆盖)
  const maxLinear = ref(0.15) // m/s
  const maxAngular = ref(0.6) // rad/s
  // 当前滑条值(用户可调)
  const linearSpeed = ref(0.1) // m/s
  const angularSpeed = ref(0.3) // rad/s

  const activeAction = ref(null) // 当前按住的动作
  const error = ref(null)
  const enabled = ref(true) // 后端二进制是否可用

  let timer = null

  async function loadStatus() {
    try {
      const s = await api.getControlStatus()
      if (s) {
        if (Number.isFinite(s.max_linear_mps)) maxLinear.value = s.max_linear_mps
        if (Number.isFinite(s.max_angular_radps)) maxAngular.value = s.max_angular_radps
        enabled.value = !!s.binary_ok
        // 把当前滑条值钳进上限
        if (linearSpeed.value > maxLinear.value) linearSpeed.value = maxLinear.value
        if (angularSpeed.value > maxAngular.value) angularSpeed.value = maxAngular.value
      }
    } catch (err) {
      // 读不到 status 不影响使用,保留默认上限
    }
  }

  function speedFor(action) {
    return action === 'turn_left' || action === 'turn_right'
      ? Number(angularSpeed.value)
      : Number(linearSpeed.value)
  }

  async function sendJog(action) {
    try {
      await api.post('/api/control/jog', { action, speed: speedFor(action) })
      error.value = null
    } catch (err) {
      error.value = err?.message || String(err)
    }
  }

  // 按住方向键:立即发一次,然后按心跳持续发,保持后端进程存活并喂看门狗。
  function press(action) {
    if (!enabled.value) {
      error.value = '控制二进制不可用 / control binary unavailable'
      return
    }
    if (activeAction.value === action) return
    activeAction.value = action
    sendJog(action)
    stopTimer()
    timer = setInterval(() => {
      if (activeAction.value) sendJog(activeAction.value)
    }, HEARTBEAT_MS)
  }

  function stopTimer() {
    if (timer) {
      clearInterval(timer)
      timer = null
    }
  }

  // 松开:停心跳并让底盘停下。
  async function release() {
    stopTimer()
    if (!activeAction.value) return
    activeAction.value = null
    try {
      await api.post('/api/control/stop', {})
    } catch (err) {
      error.value = err?.message || String(err)
    }
  }

  return {
    maxLinear,
    maxAngular,
    linearSpeed,
    angularSpeed,
    activeAction,
    error,
    enabled,
    loadStatus,
    press,
    release,
  }
})
