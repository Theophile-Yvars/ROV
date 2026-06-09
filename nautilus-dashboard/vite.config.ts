import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  optimizeDeps: {
    // Force Vite à inclure roslib et à le transformer en ESM
    include: ['roslib']
  },
  build: {
    commonjsOptions: {
      // Indique à Vite de traiter roslib comme un module CommonJS
      include: [/roslib/, /node_modules/]
    }
  }
});