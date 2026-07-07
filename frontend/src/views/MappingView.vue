<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useMappingStore } from '../stores/mapping.js'
import { useReloc2dStore } from '../stores/reloc2d.js'
import MapCanvas from '../components/MapCanvas.vue'
import RelocalizeDialog from '../components/RelocalizeDialog.vue'

const telemetry = useTelemetryStore()
const mapping = useMappingStore()
const reloc = useReloc2dStore()
const { state } = storeToRefs(telemetry)
const { busy } = storeToRefs(mapping)
const { picking } = storeToRefs(reloc)

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
      <button :disabled="busy" title="用 2D 激光对地图做重定位" @click="reloc.openDialog()">重定位 / Relocalize</button>
      <span class="map-build-status" :style="{ color: mappingColor }">{{ mappingText }}</span>
    </div>
    <div class="mapping-canvas-host">
      <!-- Mapping mode is view-only for waypoints; when picking a relocalization
           seed we enable pick-mode so a click reports a map-frame {x,y}. -->
      <div class="canvas-wrap">
        <MapCanvas :pick-mode="picking" :show-saved-points="false" @pick="reloc.onPicked($event)" />
        <div v-if="picking" class="reloc-pick-banner">
          点击地图选择重定位的大致初始位置（朝向自动搜索）
          <button class="reloc-pick-cancel" @click="reloc.cancelPick()">取消</button>
        </div>
      </div>
    </div>
    <RelocalizeDialog />
  </div>
</template>

<style scoped>
.canvas-wrap {
  position: relative;
}
.reloc-pick-banner {
  position: absolute;
  top: 12px;
  left: 50%;
  transform: translateX(-50%);
  display: flex;
  align-items: center;
  gap: 12px;
  background: rgba(37, 99, 235, 0.95);
  color: #fff;
  font-size: 13px;
  font-weight: 600;
  padding: 8px 14px;
  border-radius: 999px;
  box-shadow: 0 6px 18px rgba(15, 23, 41, 0.25);
  z-index: 20;
}
.reloc-pick-cancel {
  background: rgba(255, 255, 255, 0.2);
  border: 1px solid rgba(255, 255, 255, 0.5);
  color: #fff;
  padding: 3px 10px;
  border-radius: 999px;
  font-size: 12px;
  font-weight: 600;
}
.reloc-pick-cancel:hover {
  background: rgba(255, 255, 255, 0.32);
}
</style>
