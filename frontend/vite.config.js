import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// The Python backend (backend/base_sensor_visual_server.py) serves the REST API
// on port 18083. In dev we run Vite on 5173 and proxy /api to the backend so the
// browser sees a single origin (no CORS needed). In production the backend serves
// the built dist/ directly, so the same relative /api paths keep working.
const BACKEND_TARGET = process.env.VITE_BACKEND_TARGET || 'http://127.0.0.1:18083'

export default defineConfig({
  plugins: [vue()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    proxy: {
      '/api': {
        target: BACKEND_TARGET,
        changeOrigin: true,
      },
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
})
