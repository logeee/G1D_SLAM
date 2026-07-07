<script setup>
import { onBeforeUnmount, onMounted, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from './stores/telemetry.js'
import { useWorkflowStore } from './stores/workflow.js'
import { usePointsStore } from './stores/points.js'
import { useMappingStore } from './stores/mapping.js'
import { useRelocalizationStore } from './stores/relocalization.js'
import { useColumnHeightStore } from './stores/columnHeight.js'
import ConnectionStatus from './components/ConnectionStatus.vue'
import DashboardView from './views/DashboardView.vue'
import MappingView from './views/MappingView.vue'
import AppDialog from './components/AppDialog.vue'

const telemetry = useTelemetryStore()
const workflow = useWorkflowStore()
const points = usePointsStore()
const mapping = useMappingStore()
const relocalization = useRelocalizationStore()
const columnHeight = useColumnHeightStore()

const { state } = storeToRefs(telemetry)
const { activeView } = storeToRefs(mapping)

let columnTimer = null
let relocTimer = null

// Per-tick store updates that mirror the legacy tick() ordering. The panels
// themselves redraw reactively; here we only refresh derived nav/workflow state.
watch(state, snap => {
  if (!snap) return
  workflow.updateNavigationStatus(snap)
  workflow.updateWorkflowProgress(snap)
})

watch(activeView, view => {
  document.body.classList.toggle('mapping-active', view === 'mapping')
})

async function loadPoints() {
  try {
    await points.load()
    workflow.refreshLinkedWorkflowActions()
  } catch (err) {
    console.warn('load points failed', err)
  }
}

function showDashboard() {
  mapping.exitMapping()
}

function showMapping() {
  mapping.enterMapping()
}

onMounted(() => {
  workflow.loadWorkflowCache()
  loadPoints()
  relocalization.loadStatus()
  columnHeight.refresh({ quiet: true })
  telemetry.startPolling(500)
  columnTimer = setInterval(() => columnHeight.refresh({ quiet: true }), 1000)
  relocTimer = setInterval(() => relocalization.loadStatus(), 5000)
})

onBeforeUnmount(() => {
  telemetry.stopPolling()
  if (columnTimer) clearInterval(columnTimer)
  if (relocTimer) clearInterval(relocTimer)
  document.body.classList.remove('mapping-active')
})
</script>

<template>
  <header>
    <div class="header-tabs">
      <button class="view-tab" :class="{ active: activeView === 'dashboard' }" @click="showDashboard">Base Sensor Dashboard</button>
      <button class="view-tab" :class="{ active: activeView === 'mapping' }" @click="showMapping">Mapping Mode</button>
    </div>
    <ConnectionStatus />
  </header>
  <MappingView v-if="activeView === 'mapping'" />
  <DashboardView v-else />
  <AppDialog />
</template>
