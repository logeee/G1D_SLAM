<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { useMappingStore } from '../stores/mapping.js'
import { useReloc2dStore } from '../stores/reloc2d.js'
import MapCanvas from '../components/MapCanvas.vue'
import RelocalizeDialog from '../components/RelocalizeDialog.vue'
import CameraFeed from '../components/CameraFeed.vue'
import ControlPad from '../components/ControlPad.vue'

const telemetry = useTelemetryStore()
const mapping = useMappingStore()
const reloc = useReloc2dStore()
const { state } = storeToRefs(telemetry)
const { busy, prepared } = storeToRefs(mapping)
const { picking } = storeToRefs(reloc)

const building = computed(() => state.value?.navigation?.robot_basic_state?.is_map_building_enabled)
// 右侧栏(相机+遥控):进入「准备采集」或已在采集时显示。
const sideVisible = computed(() => prepared.value || !!building.value)
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
      <button class="primary" :disabled="busy || prepared || building" title="进入准备:显示左眼与遥控,先把机器人开到想开始的位置" @click="mapping.prepareMapping()">
        Prepare Mapping
      </button>
      <button :disabled="busy" title="Freeze the map and stop the mapping session" @click="mapping.stopMapping()">Stop Mapping</button>
      <button :disabled="busy" title="Export the current chassis map to a .stcm file" @click="mapping.saveMap()">Save Map</button>
      <button :disabled="busy" title="Load a saved .stcm map file into the chassis" @click="mapping.loadMap()">Load Map</button>
      <button :disabled="busy" title="用 2D 激光对地图做重定位" @click="reloc.openDialog()">重定位 / Relocalize</button>
      <span class="map-build-status" :style="{ color: mappingColor }">{{ mappingText }}</span>
    </div>
    <div class="mapping-canvas-host" :class="{ 'with-side': sideVisible }">
      <!-- Mapping mode is view-only for waypoints; when picking a relocalization
           seed we enable pick-mode so a click reports a map-frame {x,y}. -->
      <div class="canvas-wrap">
        <MapCanvas :pick-mode="picking" :show-saved-points="false" @pick="reloc.onPicked($event)" />
        <div v-if="picking" class="reloc-pick-banner">
          点击地图选择重定位的大致初始位置（朝向自动搜索）
          <button class="reloc-pick-cancel" @click="reloc.cancelPick()">取消</button>
        </div>
      </div>
      <!-- 准备采集或采集中显示右侧:上=左眼相机,下=底盘遥控。 -->
      <aside v-if="sideVisible" class="mapping-side">
        <div class="side-camera">
          <div class="side-title">左眼 / Left eye</div>
          <div class="side-camera-body">
            <CameraFeed :active="sideVisible" />
          </div>
        </div>
        <div class="side-control">
          <div class="side-title">
            <span>底盘遥控 / Jog</span>
            <span v-if="building" class="side-badge">采集中 / Mapping</span>
          </div>
          <!-- 准备阶段:开到起点后点这里真正开始建图(会清空底盘当前地图并确认)。 -->
          <div v-if="prepared && !building" class="confirm-start-bar">
            <button class="confirm-start-btn" :disabled="busy" @click="mapping.startMapping()">
              ✓ 确认开始采集 / Confirm Start
            </button>
            <button class="confirm-cancel-btn" :disabled="busy" title="退出准备" @click="mapping.cancelPrepare()">
              取消
            </button>
          </div>
          <div class="side-control-body">
            <ControlPad />
          </div>
        </div>
      </aside>
    </div>
    <RelocalizeDialog />
  </div>
</template>

<style scoped>
.canvas-wrap {
  position: relative;
}
/* 采集中:左右分栏。左=地图,右=上下分(相机/控制)。 */
.mapping-canvas-host.with-side {
  display: flex;
  gap: 12px;
}
.mapping-canvas-host.with-side .canvas-wrap {
  flex: 1 1 auto;
  width: auto;
  min-width: 0;
}
.mapping-side {
  flex: 0 0 34%;
  max-width: 460px;
  min-width: 300px;
  display: flex;
  flex-direction: column;
  gap: 12px;
  min-height: 0;
}
.side-camera,
.side-control {
  display: flex;
  flex-direction: column;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: #fbfcfe;
  overflow: hidden;
  min-height: 0;
}
.side-camera {
  flex: 1 1 50%;
}
.side-control {
  flex: 1 1 50%;
}
.side-title {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
  padding: 8px 12px;
  font-size: 13px;
  font-weight: 600;
  color: var(--text);
  border-bottom: 1px solid var(--line);
  background: #f4f7fb;
}
.side-badge {
  font-size: 11px;
  font-weight: 700;
  color: #fff;
  background: var(--ok, #16a34a);
  padding: 2px 8px;
  border-radius: 999px;
}
.confirm-start-bar {
  display: flex;
  gap: 8px;
  padding: 6px 10px;
  border-bottom: 1px solid var(--line);
  background: #eefaf2;
}
.confirm-start-btn {
  flex: 1 1 auto;
  padding: 6px 12px;
  font-size: 13px;
  font-weight: 700;
  color: #fff;
  background: var(--ok, #16a34a);
  border: 1px solid var(--ok, #16a34a);
  border-radius: 7px;
  cursor: pointer;
}
.confirm-start-btn:hover:not(:disabled) {
  filter: brightness(1.08);
}
.confirm-start-btn:disabled,
.confirm-cancel-btn:disabled {
  opacity: 0.55;
  cursor: not-allowed;
}
.confirm-cancel-btn {
  flex: 0 0 auto;
  padding: 6px 12px;
  font-size: 12px;
  font-weight: 600;
  color: var(--text);
  background: #fff;
  border: 1px solid var(--line);
  border-radius: 7px;
  cursor: pointer;
}
.confirm-cancel-btn:hover:not(:disabled) {
  background: #eef1f6;
}
.side-camera-body {
  flex: 1 1 auto;
  min-height: 0;
  padding: 8px;
}
.side-control-body {
  flex: 1 1 auto;
  min-height: 0;
  overflow: hidden;
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
