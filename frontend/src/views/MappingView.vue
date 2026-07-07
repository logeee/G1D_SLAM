<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useMappingStore } from '../stores/mapping.js'
import MapCanvas from '../components/MapCanvas.vue'

const telemetry = useTelemetryStore()
const mapping = useMappingStore()
const { state } = storeToRefs(telemetry)
const { busy } = storeToRefs(mapping)

const building = computed(() => state.value?.navigation?.robot_basic_state?.is_map_building_enabled)
const mappingText = computed(() => {
  if (building.value === null || building.value === undefined) return 'Mapping: --'
  return building.value ? 'Mapping: ON' : 'Mapping: OFF'
})
const mappingColor = computed(() => {
  if (building.value === null || building.value === undefined) return 'var(--muted)'
  return building.value ? 'var(--ok)' : 'var(--muted)'
})
</script>

<template>
  <div id="mappingView" class="view-active">
    <div class="mapping-toolbar">
      <button class="primary" :disabled="busy" title="Clear the chassis map and start a fresh mapping session" @click="mapping.startMapping()">
        Start Mapping
      </button>
      <button :disabled="busy" title="Freeze the map and stop the mapping session" @click="mapping.stopMapping()">Stop Mapping</button>
      <button :disabled="busy" title="Export the current chassis map to a .stcm file" @click="mapping.saveMap()">Save Map</button>
      <button :disabled="busy" title="Load a saved .stcm map file into the chassis" @click="mapping.loadMap()">Load Map</button>
      <span class="map-build-status" :style="{ color: mappingColor }">{{ mappingText }}</span>
    </div>
    <div class="mapping-canvas-host">
      <!-- Mapping mode is view-only: no click-to-add waypoint here.
           (Dashboard 的 SLAM Map 仍保留打点，见 MapPanel.vue) -->
      <div class="canvas-wrap"><MapCanvas /></div>
    </div>
  </div>
</template>
