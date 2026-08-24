import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  // Set base to repo name for GitHub Pages deployment
  // Change this to your actual repo name, e.g. "/orbital-trajectory-simulator/"
  base: "/orbital-trajectory-simulator/",
  build: {
    outDir: "dist",
    sourcemap: true,
  },
});
