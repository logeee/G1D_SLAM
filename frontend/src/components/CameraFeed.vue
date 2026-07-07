<script setup>
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'

const props = defineProps({
  // 是否处于激活状态(采集中)。为 false 时断开流,避免后台占用相机。
  active: { type: Boolean, default: true },
})

// MJPEG 流地址(后端 /api/camera/left_eye)。加时间戳避免缓存。
const src = ref('')
const failed = ref(false)

function connect() {
  failed.value = false
  src.value = `/api/camera/left_eye?ts=${Date.now()}`
}
function disconnect() {
  src.value = ''
}
function onError() {
  failed.value = true
}
function retry() {
  connect()
}

watch(
  () => props.active,
  (on) => {
    if (on) connect()
    else disconnect()
  },
)

onMounted(() => {
  if (props.active) connect()
})
onBeforeUnmount(disconnect)
</script>

<template>
  <div class="camera-feed">
    <img
      v-if="src && !failed"
      :src="src"
      class="camera-img"
      alt="left eye"
      @error="onError"
    />
    <div v-else class="camera-placeholder">
      <div class="camera-ph-title">左眼画面不可用</div>
      <div class="camera-ph-sub">Left-eye camera unavailable</div>
      <button class="camera-retry" @click="retry">重试</button>
    </div>
  </div>
</template>

<style scoped>
.camera-feed {
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #0b1220;
  border-radius: 10px;
  overflow: hidden;
}
.camera-img {
  width: 100%;
  height: 100%;
  object-fit: cover;
  display: block;
}
.camera-placeholder {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6px;
  color: #93a1b5;
}
.camera-ph-title {
  font-size: 14px;
  font-weight: 600;
}
.camera-ph-sub {
  font-size: 12px;
  color: #6b7688;
}
.camera-retry {
  margin-top: 6px;
  background: rgba(255, 255, 255, 0.12);
  border: 1px solid rgba(255, 255, 255, 0.28);
  color: #dbe4f0;
  padding: 4px 14px;
  border-radius: 999px;
  font-size: 12px;
  cursor: pointer;
}
.camera-retry:hover {
  background: rgba(255, 255, 255, 0.2);
}
</style>
