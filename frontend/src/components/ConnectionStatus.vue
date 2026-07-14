<script setup>
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'

const store = useTelemetryStore()
const { online, statusText } = storeToRefs(store)
</script>

<template>
  <div class="status">
    <span class="dot" :class="{ ok: online }"></span>
    <span>{{ statusText }}</span>
  </div>
</template>

<style scoped>
.status {
  display: flex;
  align-items: center;
  gap: 10px;
  color: var(--muted);
  white-space: nowrap;
}
.dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background: var(--warn);
}
.dot.ok {
  background: var(--ok);
}
</style>
