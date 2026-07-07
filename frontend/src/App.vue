<script setup>
import { onMounted, onBeforeUnmount } from 'vue'
import { useTelemetryStore } from './stores/telemetry.js'
import ConnectionStatus from './components/ConnectionStatus.vue'
import LaserScanPanel from './components/LaserScanPanel.vue'
import SensorsPanel from './components/SensorsPanel.vue'

const store = useTelemetryStore()

onMounted(() => store.startPolling(500))
onBeforeUnmount(() => store.stopPolling())
</script>

<template>
  <header>
    <h1>Base Sensor Dashboard</h1>
    <ConnectionStatus />
  </header>
  <main>
    <div class="migration-note">
      Vue3 迁移样板：本页目前只迁移了 <strong>Laser Scan</strong> 与
      <strong>Ultrasonic / Bumper Sensors</strong> 两个面板，用于验证「命令式 Canvas + 响应式数据」在 Vue3
      下可行。其余面板（地图/点云/动作链/点位库/故障快照等）待确认后再逐步迁移。
    </div>
    <LaserScanPanel />
    <SensorsPanel />
  </main>
</template>
