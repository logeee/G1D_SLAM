<script setup>
import { computed, nextTick, onBeforeUnmount, ref, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useDialogStore } from '../stores/dialog.js'

const dialog = useDialogStore()
const { visible, options } = storeToRefs(dialog)

const inputEl = ref(null)
const inputValue = ref('')

const isPrompt = computed(() => options.value.type === 'prompt')
const isSelect = computed(() => options.value.type === 'select')
const isAlert = computed(() => options.value.type === 'alert')
const hasInput = computed(() => isPrompt.value || isSelect.value)

function onWindowKeydown(ev) {
  if (!visible.value) return
  if (ev.key === 'Escape') {
    ev.preventDefault()
    onCancel()
  } else if (ev.key === 'Enter' && !isPrompt.value) {
    // For prompt, the input's own @keydown.enter handles submit.
    ev.preventDefault()
    onConfirm()
  }
}

watch(visible, async open => {
  if (open) {
    inputValue.value = options.value.defaultValue || ''
    window.addEventListener('keydown', onWindowKeydown)
    if (hasInput.value) {
      await nextTick()
      const el = inputEl.value
      if (el) {
        el.focus()
        if (isPrompt.value && el.select) el.select()
      }
    }
  } else {
    window.removeEventListener('keydown', onWindowKeydown)
  }
})

onBeforeUnmount(() => window.removeEventListener('keydown', onWindowKeydown))

function onConfirm() {
  dialog.accept(hasInput.value ? inputValue.value : true)
}

function onCancel() {
  dialog.cancel()
}

function onOverlayMouseDown(ev) {
  // Only cancel when the backdrop itself is pressed (not the card).
  if (ev.target === ev.currentTarget) onCancel()
}
</script>

<template>
  <Teleport to="body">
    <Transition name="app-dialog">
      <div
        v-if="visible"
        class="app-dialog-overlay"
        @mousedown="onOverlayMouseDown"
      >
        <div class="app-dialog-card" role="dialog" aria-modal="true">
          <div v-if="options.title" class="app-dialog-title">{{ options.title }}</div>
          <div class="app-dialog-message">{{ options.message }}</div>

          <input
            v-if="isPrompt"
            ref="inputEl"
            v-model="inputValue"
            class="app-dialog-input"
            type="text"
            :placeholder="options.placeholder"
            @keydown.enter.prevent="onConfirm"
          />

          <select
            v-else-if="isSelect"
            ref="inputEl"
            v-model="inputValue"
            class="app-dialog-input app-dialog-select"
          >
            <option v-for="item in options.items" :key="item.value" :value="item.value">
              {{ item.label }}
            </option>
          </select>

          <div class="app-dialog-actions">
            <button v-if="!isAlert" type="button" @click="onCancel">{{ options.cancelText }}</button>
            <button
              type="button"
              :class="options.danger ? 'danger' : 'primary'"
              @click="onConfirm"
            >
              {{ options.confirmText }}
            </button>
          </div>
        </div>
      </div>
    </Transition>
  </Teleport>
</template>

<style scoped>
.app-dialog-overlay {
  position: fixed;
  inset: 0;
  z-index: 1000;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 24px;
  background: rgba(15, 23, 41, 0.45);
  backdrop-filter: blur(2px);
}
.app-dialog-card {
  width: min(520px, 100%);
  max-height: calc(100vh - 48px);
  overflow: auto;
  background: var(--panel);
  color: var(--text);
  border: 1px solid var(--line);
  border-radius: 12px;
  box-shadow: 0 18px 48px rgba(15, 23, 41, 0.28);
  padding: 20px 22px;
}
.app-dialog-title {
  font-size: 16px;
  font-weight: 700;
  margin-bottom: 10px;
}
.app-dialog-message {
  font-size: 14px;
  line-height: 1.55;
  color: var(--text);
  white-space: pre-line;
  word-break: break-word;
}
.app-dialog-input {
  margin-top: 14px;
  width: 100%;
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 9px 11px;
  font-size: 14px;
  color: var(--text);
  background: #fff;
}
.app-dialog-input:focus {
  outline: none;
  border-color: var(--blue);
  box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.18);
}
.app-dialog-select {
  cursor: pointer;
  appearance: none;
  background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='8' viewBox='0 0 12 8'%3E%3Cpath fill='%2366758a' d='M1 1l5 5 5-5'/%3E%3C/svg%3E");
  background-repeat: no-repeat;
  background-position: right 12px center;
  padding-right: 32px;
}
.app-dialog-actions {
  margin-top: 20px;
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}
.app-dialog-enter-active,
.app-dialog-leave-active {
  transition: opacity 0.15s ease;
}
.app-dialog-enter-from,
.app-dialog-leave-to {
  opacity: 0;
}
</style>
