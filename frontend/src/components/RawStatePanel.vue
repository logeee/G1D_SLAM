<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useTelemetryStore } from '../stores/telemetry.js'

const telemetry = useTelemetryStore()
const { state } = storeToRefs(telemetry)

const rawText = computed(() => {
  const s = state.value
  if (!s) return '{}'
  try {
    return JSON.stringify(
      {
        freshness_s: s.freshness_s,
        seq: s.seq,
        odom: s.odom,
        scan: s.scan
          ? { frame_id: s.scan.frame_id, min_range: s.scan.min_range, valid_count: s.scan.valid_count, count: s.scan.count }
          : null,
        point_cloud: s.point_cloud
          ? {
              topic: s.point_cloud.topic,
              frame_id: s.point_cloud.frame_id,
              total_points: s.point_cloud.total_points,
              sampled_points: s.point_cloud.sampled_points,
              bounds: s.point_cloud.bounds,
            }
          : null,
        sensors: s.sensors,
        map: s.map
          ? { frame_id: s.map.frame_id, width: s.map.width, height: s.map.height, resolution: s.map.resolution, origin: s.map.origin }
          : null,
        navigation: s.navigation,
        arm_control: s.arm_control,
        fault_snapshots: s.fault_snapshots ? s.fault_snapshots.slice(-5) : [],
      },
      null,
      2,
    )
  } catch (err) {
    return String(err)
  }
})
</script>

<template>
  <section>
    <div class="panel-head">
      <h2>Raw State</h2>
      <span class="meta">/api/state</span>
    </div>
    <pre class="raw-state">{{ rawText }}</pre>
  </section>
</template>
