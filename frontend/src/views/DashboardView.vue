<script setup>
import MapPanel from '../components/MapPanel.vue'
import PointsPanel from '../components/PointsPanel.vue'
import LaserScanPanel from '../components/LaserScanPanel.vue'
import SensorsPanel from '../components/SensorsPanel.vue'
import FaultSnapshotsPanel from '../components/FaultSnapshotsPanel.vue'
import PointCloudPanel from '../components/PointCloudPanel.vue'
import RawStatePanel from '../components/RawStatePanel.vue'
</script>

<template>
  <main>
    <MapPanel />
    <PointsPanel />
    <LaserScanPanel />
    <SensorsPanel />
    <FaultSnapshotsPanel />
    <PointCloudPanel />
    <RawStatePanel />
  </main>
</template>
