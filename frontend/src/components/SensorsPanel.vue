<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'
import { fmt } from '../utils/format.js'

const store = useTelemetryStore()
const { state } = storeToRefs(store)

const sensors = computed(() => state.value?.sensors?.items || [])
const meta = computed(() =>
  sensors.value.length
    ? `${sensors.value.length} sensors, age ${fmt(state.value?.freshness_s?.sensors, 2, 's')}`
    : 'waiting',
)

function sensorValue(s) {
  return s.raw_value_is_finite ? fmt(s.value, 3) : 'inf'
}
</script>

<template>
  <section>
    <div class="panel-head">
      <h2>Ultrasonic / Bumper Sensors</h2>
      <span class="meta">{{ meta }}</span>
    </div>
    <div class="sensor-grid">
      <template v-if="sensors.length">
        <div v-for="s in sensors" :key="s.id" class="sensor-card" :class="{ hit: s.is_in_impact }">
          <div class="sensor-title">
            <span>#{{ s.id }} {{ s.sensor_type_name }}</span>
            <span>{{ s.is_in_impact ? 'HIT' : 'OK' }}</span>
          </div>
          <div class="kv">
            <span>value</span><strong>{{ sensorValue(s) }}</strong>
            <span>impact</span><strong>{{ s.impact_type_name }}</strong>
            <span>pose</span
            ><strong>x={{ fmt(s.pose.x, 3) }}, y={{ fmt(s.pose.y, 3) }}, z={{ fmt(s.pose.z, 3) }}</strong>
            <span>freq</span><strong>{{ fmt(s.refresh_freq, 1, 'Hz') }}</strong>
          </div>
        </div>
      </template>
      <div v-else class="sensor-card">waiting for sensors</div>
    </div>
  </section>
</template>
