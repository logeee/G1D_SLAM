<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useReloc2dStore } from '../stores/reloc2d.js'

const reloc = useReloc2dStore()
const { open, step, running, result, error, applied, applyError, saveIntervalSec, config, configBusy } = storeToRefs(reloc)

const r = computed(() => result.value)
const accepted = computed(() => r.value && r.value.accepted)

function onOverlayMouseDown(ev) {
  if (ev.target === ev.currentTarget && !running.value) reloc.close()
}
</script>

<template>
  <Teleport to="body">
    <!-- 选择方式:居中模态(遮罩)。 -->
    <Transition name="app-dialog">
      <div v-if="open && step === 'choose'" class="app-dialog-overlay" @mousedown="onOverlayMouseDown">
        <div class="app-dialog-card reloc-card" role="dialog" aria-modal="true">
          <div class="app-dialog-title">重定位 / Relocalization</div>
          <div class="reloc-hint">选择一种重定位方式（基于 2D 激光 ↔ 地图配准）：</div>
          <div class="reloc-methods">
            <button class="reloc-method" :disabled="running" @click="reloc.runGlobal()">
              <div class="reloc-method-title">雷达全局匹配</div>
              <div class="reloc-method-desc">无需初值，全图搜索。较慢但不依赖先验。</div>
            </button>
            <button class="reloc-method" :disabled="running" @click="reloc.runJson()">
              <div class="reloc-method-title">双阶段 ICP · 上次位姿</div>
              <div class="reloc-method-desc">以定时保存的最近位姿为初值（每 10s 自动保存）。</div>
            </button>
            <button class="reloc-method" :disabled="running" @click="reloc.beginClickPick()">
              <div class="reloc-method-title">双阶段 ICP · 手动选点</div>
              <div class="reloc-method-desc">在地图上点一个大致位置作为初值，朝向自动搜索。</div>
            </button>
          </div>
          <div v-if="running" class="reloc-running">配准中，请稍候…</div>

          <div class="reloc-config">
            <div class="reloc-config-row">
              <span>位姿自动保存间隔</span>
              <input
                v-model.number="saveIntervalSec"
                class="reloc-config-input"
                type="number"
                min="0"
                step="1"
                :disabled="configBusy"
              />
              <span class="reloc-config-unit">秒</span>
              <button class="reloc-config-apply" :disabled="configBusy" @click="reloc.applyInterval()">应用</button>
            </div>
            <div class="reloc-config-hint">
              供「上次位姿」初值使用；填 0 关闭自动保存。
              <template v-if="config && config.last_pose">
                最近保存：{{ config.last_pose.saved_at || '--' }}
              </template>
              <template v-else>暂无已保存位姿</template>
            </div>
          </div>

          <div class="app-dialog-actions">
            <button type="button" :disabled="running" @click="reloc.close()">关闭</button>
          </div>
        </div>
      </div>
    </Transition>

    <!-- 结果:右侧角落面板,不带遮罩,地图与候选位姿箭头完整可见,便于确认后再应用。 -->
    <Transition name="app-dialog">
      <div v-if="open && step === 'result'" class="reloc-result-panel" role="dialog">
        <div class="app-dialog-title">重定位结果 / Result</div>
        <div v-if="running" class="reloc-running">处理中，请稍候…</div>
        <template v-else>
          <div v-if="error" class="reloc-error">配准失败：{{ error }}</div>
          <template v-else-if="r">
            <div class="reloc-preview-tip">
              地图上<b :style="{ color: accepted ? '#16a34a' : '#f97316' }">{{ accepted ? '绿色' : '橙色' }}</b>箭头为候选位姿，请先看落点与朝向再决定。
            </div>
            <div class="reloc-badge" :class="accepted ? 'ok' : 'warn'">
              {{ accepted ? '✓ 达标（高可信）' : '⚠ 未达标（谨慎使用）' }}
            </div>
            <div class="reloc-result">
              <div><span>位姿 x / y</span><b>{{ r.pose.x }} , {{ r.pose.y }} m</b></div>
              <div><span>朝向 yaw</span><b>{{ r.pose.yaw_deg }}°</b></div>
              <div><span>RMSE</span><b>{{ r.rmse === null ? '--' : r.rmse + ' m' }}</b></div>
              <div><span>fitness（内点占比）</span><b>{{ r.fitness }}</b></div>
              <div v-if="r.spread"><span>散布 pos / yaw</span><b>{{ r.spread.pos_rms }} m / {{ r.spread.yaw_std_deg }}°</b></div>
              <div><span>方式 / 起点数</span><b>{{ r.input_method }} / {{ r.n_starts ?? '--' }}</b></div>
              <div><span>扫描 / 地图点</span><b>{{ r.n_scan }} / {{ r.n_map }}</b></div>
            </div>
            <div v-if="applied" class="reloc-applied">✓ 已应用到底盘（已触发定位恢复）</div>
            <div v-if="applyError" class="reloc-error" style="margin-top:10px">应用失败：{{ applyError }}</div>
          </template>
        </template>

        <div class="app-dialog-actions">
          <button v-if="!running" type="button" @click="reloc.openDialog()">重试 / 换方式</button>
          <button type="button" :disabled="running" @click="reloc.close()">关闭</button>
          <button
            v-if="r && !running && !applied"
            type="button"
            class="primary"
            @click="reloc.applyToChassis()"
          >
            应用到底盘
          </button>
        </div>
      </div>
    </Transition>
  </Teleport>
</template>

<style scoped>
.reloc-card {
  width: min(560px, 100%);
}
/* 结果面板:固定在右上角,不遮挡地图,可看到候选位姿箭头再决定。 */
.reloc-result-panel {
  position: fixed;
  top: 84px;
  right: 20px;
  z-index: 60;
  width: min(360px, calc(100vw - 40px));
  max-height: calc(100vh - 120px);
  overflow: auto;
  background: #fff;
  border: 1px solid var(--line);
  border-radius: 12px;
  box-shadow: 0 12px 40px rgba(15, 23, 41, 0.28);
  padding: 16px 18px;
}
.reloc-preview-tip {
  font-size: 12px;
  color: var(--muted);
  margin-bottom: 10px;
  line-height: 1.5;
}
.reloc-hint {
  font-size: 13px;
  color: var(--muted);
  margin-bottom: 12px;
}
.reloc-methods {
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.reloc-method {
  text-align: left;
  padding: 12px 14px;
  border: 1px solid var(--line);
  border-radius: 10px;
  background: #f8fafc;
  cursor: pointer;
  transition: border-color 0.12s ease, background 0.12s ease;
}
.reloc-method:hover:not(:disabled) {
  border-color: var(--blue);
  background: #eff5ff;
}
.reloc-method:disabled {
  opacity: 0.55;
  cursor: not-allowed;
}
.reloc-method-title {
  font-weight: 700;
  font-size: 14px;
  color: var(--text);
}
.reloc-method-desc {
  font-size: 12px;
  color: var(--muted);
  margin-top: 3px;
  font-weight: 400;
}
.reloc-running {
  margin-top: 14px;
  font-size: 13px;
  color: var(--blue);
}
.reloc-config {
  margin-top: 16px;
  padding-top: 14px;
  border-top: 1px dashed var(--line);
}
.reloc-config-row {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  color: var(--text);
}
.reloc-config-input {
  width: 72px;
  border: 1px solid var(--line);
  border-radius: 6px;
  padding: 5px 8px;
  font-size: 13px;
  color: var(--text);
  background: #fff;
}
.reloc-config-input:focus {
  outline: none;
  border-color: var(--blue);
  box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.18);
}
.reloc-config-unit {
  color: var(--muted);
}
.reloc-config-apply {
  margin-left: auto;
  padding: 5px 12px;
  font-size: 12px;
}
.reloc-config-hint {
  margin-top: 6px;
  font-size: 12px;
  color: var(--muted);
}
.reloc-error {
  font-size: 13px;
  color: #b91c1c;
  background: #fff1f2;
  border: 1px solid #fecaca;
  border-radius: 8px;
  padding: 10px 12px;
  white-space: pre-line;
}
.reloc-badge {
  display: inline-block;
  font-size: 13px;
  font-weight: 700;
  padding: 5px 12px;
  border-radius: 999px;
  margin-bottom: 12px;
}
.reloc-badge.ok {
  color: #166534;
  background: #dcfce7;
  border: 1px solid #86efac;
}
.reloc-badge.warn {
  color: #92400e;
  background: #fef3c7;
  border: 1px solid #fcd34d;
}
.reloc-result {
  display: grid;
  gap: 8px;
}
.reloc-result > div {
  display: flex;
  justify-content: space-between;
  align-items: baseline;
  font-size: 13px;
  border-bottom: 1px dashed var(--line);
  padding-bottom: 6px;
}
.reloc-result span {
  color: var(--muted);
}
.reloc-result b {
  color: var(--text);
  font-variant-numeric: tabular-nums;
}
.reloc-applied {
  margin-top: 12px;
  font-size: 13px;
  font-weight: 700;
  color: #166534;
  background: #dcfce7;
  border: 1px solid #86efac;
  border-radius: 8px;
  padding: 8px 12px;
}
</style>
