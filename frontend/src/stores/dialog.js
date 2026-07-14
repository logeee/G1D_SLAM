import { defineStore } from 'pinia'
import { ref } from 'vue'

// Promise-based dialog service that replaces the native window.confirm /
// prompt / alert. A single <AppDialog /> mounted at the app root renders the
// current request; actions await the returned promise.
//
//   const ok    = await dialog.confirm('确定吗？')          // -> boolean
//   const name  = await dialog.prompt('名称', 'default')    // -> string | null (null = 取消)
//   await dialog.alert('已完成')                            // -> true
export const useDialogStore = defineStore('dialog', () => {
  const visible = ref(false)
  const options = ref({})
  let resolver = null

  function open(opts) {
    // If a dialog is already open, resolve it as cancelled before replacing.
    if (resolver) {
      const prev = resolver
      resolver = null
      prev(cancelValueFor(options.value.type))
    }
    options.value = {
      type: 'confirm',
      title: '',
      message: '',
      defaultValue: '',
      placeholder: '',
      items: [], // for type === 'select': [{ value, label }]
      confirmText: '确定',
      cancelText: '取消',
      danger: false,
      ...opts,
    }
    visible.value = true
    return new Promise(resolve => {
      resolver = resolve
    })
  }

  function settle(value) {
    visible.value = false
    const r = resolver
    resolver = null
    if (r) r(value)
  }

  function confirm(message, opts = {}) {
    return open({ ...opts, type: 'confirm', message })
  }

  function prompt(message, defaultValue = '', opts = {}) {
    return open({ ...opts, type: 'prompt', message, defaultValue })
  }

  // items: [{ value, label }]; resolves the chosen value, or null on cancel.
  function select(message, items = [], opts = {}) {
    const defaultValue = opts.defaultValue ?? (items[0] ? items[0].value : '')
    return open({ ...opts, type: 'select', message, items, defaultValue })
  }

  function alert(message, opts = {}) {
    return open({ ...opts, type: 'alert', message })
  }

  // Value returned when a dialog is dismissed (Esc / 取消 / backdrop).
  function cancelValueFor(type) {
    if (type === 'prompt' || type === 'select') return null
    return type === 'alert' ? true : false
  }

  // Called by AppDialog.
  function accept(inputValue) {
    const type = options.value.type
    settle(type === 'prompt' || type === 'select' ? inputValue : true)
  }

  function cancel() {
    settle(cancelValueFor(options.value.type))
  }

  return { visible, options, confirm, prompt, select, alert, accept, cancel }
})
