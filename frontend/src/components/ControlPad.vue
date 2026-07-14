<script setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { storeToRefs } from 'pinia'
import { useControlStore } from '../stores/control.js'

const control = useControlStore()
const { linearSpeed, angularSpeed, maxLinear, maxAngular, activeAction, error, enabled } =
  storeToRefs(control)

// 键盘控制开关(默认关,避免误触)。
const keyboardOn = ref(false)
// 记录当前由哪个物理键触发运动,只有该键抬起才停。
let activeKey = null

const KEY_MAP = {
  ArrowUp: 'forward',
  ArrowDown: 'back',
  ArrowLeft: 'turn_left',
  ArrowRight: 'turn_right',
  w: 'forward',
  W: 'forward',
  s: 'back',
  S: 'back',
  a: 'turn_left',
  A: 'turn_left',
  d: 'turn_right',
  D: 'turn_right',
}

// 按住/松开:pointer 事件覆盖鼠标与触摸;离开元素或抬起都算松开。
function onPress(action, ev) {
  ev.preventDefault()
  control.press(action)
}
function onRelease(ev) {
  if (ev) ev.preventDefault()
  control.release()
}

function toggleKeyboard() {
  keyboardOn.value = !keyboardOn.value
  if (!keyboardOn.value) {
    activeKey = null
    control.release()
  }
}

// 若焦点在输入框/下拉/可编辑区(如弹窗输入),不拦截键盘。
function typingInField(ev) {
  const el = ev.target
  if (!el) return false
  const tag = el.tagName
  return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el.isContentEditable
}

function onKeyDown(ev) {
  if (!keyboardOn.value || !enabled.value) return
  if (typingInField(ev)) return
  const action = KEY_MAP[ev.key]
  if (!action) return
  ev.preventDefault()
  activeKey = ev.key
  control.press(action)
}

function onKeyUp(ev) {
  if (!keyboardOn.value) return
  const action = KEY_MAP[ev.key]
  if (!action) return
  ev.preventDefault()
  // 只有抬起的是当前触发键才停(避免多键时误停)。
  if (activeKey === ev.key) {
    activeKey = null
    control.release()
  }
}

// 安全:窗口失焦、切到后台、卸载时都立即停。
function safetyStop() {
  activeKey = null
  control.release()
}
function onVisibility() {
  if (document.hidden) {
    activeKey = null
    control.release()
  }
}

onMounted(() => {
  control.loadStatus()
  window.addEventListener('blur', safetyStop)
  window.addEventListener('pointerup', onRelease)
  window.addEventListener('keydown', onKeyDown)
  window.addEventListener('keyup', onKeyUp)
  document.addEventListener('visibilitychange', onVisibility)
})
onBeforeUnmount(() => {
  control.release()
  window.removeEventListener('blur', safetyStop)
  window.removeEventListener('pointerup', onRelease)
  window.removeEventListener('keydown', onKeyDown)
  window.removeEventListener('keyup', onKeyUp)
  document.removeEventListener('visibilitychange', onVisibility)
})

function btnClass(action) {
  return ['jog-btn', { 'jog-btn-active': activeAction.value === action }]
}
</script>

<template>
  <div class="control-pad">
    <div v-if="!enabled" class="control-warn">控制不可用（SDK 二进制未找到）</div>

    <div class="kbd-row">
      <button
        class="kbd-toggle"
        :class="{ 'kbd-on': keyboardOn }"
        :disabled="!enabled"
        @click="toggleKeyboard"
      >
        ⌨ 键盘控制：{{ keyboardOn ? '开' : '关' }}
      </button>
      <span v-if="keyboardOn" class="kbd-hint">↑↓←→ / WASD</span>
    </div>

    <div class="jog-grid">
      <button
        :class="btnClass('forward')"
        :disabled="!enabled"
        @pointerdown="onPress('forward', $event)"
        @pointerleave="onRelease"
        @contextmenu.prevent
        style="grid-area: up"
      >▲<span class="jog-label">前进</span></button>

      <button
        :class="btnClass('turn_left')"
        :disabled="!enabled"
        @pointerdown="onPress('turn_left', $event)"
        @pointerleave="onRelease"
        @contextmenu.prevent
        style="grid-area: left"
      >◀<span class="jog-label">左转</span></button>

      <button
        :class="btnClass('turn_right')"
        :disabled="!enabled"
        @pointerdown="onPress('turn_right', $event)"
        @pointerleave="onRelease"
        @contextmenu.prevent
        style="grid-area: right"
      >▶<span class="jog-label">右转</span></button>

      <button
        :class="btnClass('back')"
        :disabled="!enabled"
        @pointerdown="onPress('back', $event)"
        @pointerleave="onRelease"
        @contextmenu.prevent
        style="grid-area: down"
      >▼<span class="jog-label">后退</span></button>
    </div>

    <div class="speed-row">
      <label class="speed-item">
        <span class="speed-head">
          <span>线速度 Linear</span>
          <span class="speed-val">{{ Number(linearSpeed).toFixed(2) }} m/s</span>
        </span>
        <input
          type="range"
          min="0.02"
          :max="maxLinear"
          step="0.01"
          v-model.number="linearSpeed"
        />
      </label>

      <label class="speed-item">
        <span class="speed-head">
          <span>角速度 Angular</span>
          <span class="speed-val">{{ Number(angularSpeed).toFixed(2) }} rad/s</span>
        </span>
        <input
          type="range"
          min="0.05"
          :max="maxAngular"
          step="0.01"
          v-model.number="angularSpeed"
        />
      </label>
    </div>

    <div v-if="error" class="control-err">{{ error }}</div>
  </div>
</template>

<style scoped>
.control-pad {
  display: flex;
  flex-direction: column;
  gap: 8px;
  height: 100%;
  padding: 8px 10px;
  box-sizing: border-box;
  overflow: hidden;
}
.control-warn {
  color: var(--warn, #d97706);
  font-size: 13px;
  font-weight: 600;
}
.kbd-row {
  display: flex;
  align-items: center;
  gap: 8px;
  flex: 0 0 auto;
}
.kbd-toggle {
  padding: 4px 10px;
  font-size: 12px;
  font-weight: 600;
  color: var(--text, #152235);
  background: #fff;
  border: 1px solid var(--line, #d7deea);
  border-radius: 999px;
  cursor: pointer;
}
.kbd-toggle:hover:not(:disabled) {
  background: #eef1f6;
}
.kbd-toggle.kbd-on {
  color: #fff;
  background: var(--accent, #2563eb);
  border-color: var(--accent, #2563eb);
}
.kbd-toggle:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}
.kbd-hint {
  font-size: 11px;
  color: var(--muted, #6b7688);
}
.jog-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  grid-template-rows: repeat(2, 1fr);
  grid-template-areas:
    '. up .'
    'left down right';
  gap: 6px;
  flex: 1 1 auto;
  min-height: 0;
  max-width: 320px;
  margin: 0 auto 4px;
  width: 100%;
}
.jog-btn {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 0;
  font-size: 18px;
  line-height: 1;
  font-weight: 700;
  color: var(--fg, #152235);
  background: var(--panel-2, #eef2f9);
  border: 1px solid var(--border, #d7deea);
  border-radius: 10px;
  cursor: pointer;
  user-select: none;
  touch-action: none;
  transition: background 0.08s, transform 0.08s;
}
.jog-btn:hover:not(:disabled) {
  background: var(--panel-3, #e2e8f4);
}
.jog-btn-active {
  background: var(--accent, #2563eb);
  color: #fff;
  transform: scale(0.97);
}
.jog-btn:disabled {
  opacity: 0.45;
  cursor: not-allowed;
}
.jog-label {
  font-size: 10px;
  font-weight: 600;
  margin-top: 1px;
}
.speed-row {
  display: flex;
  flex-direction: column;
  gap: 6px;
  flex: 0 0 auto;
}
.speed-item {
  display: flex;
  flex-direction: column;
  gap: 2px;
  font-size: 12px;
}
.speed-head {
  display: flex;
  justify-content: space-between;
  font-weight: 600;
}
.speed-val {
  color: var(--accent, #2563eb);
}
.speed-item input[type='range'] {
  width: 100%;
  margin: 0;
}
.control-err {
  color: var(--err, #dc2626);
  font-size: 12px;
  text-align: center;
}
</style>
