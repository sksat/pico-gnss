import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// React ダッシュボードを web/ からビルドし、Node ブリッジが配る ../public へ出力する。
export default defineConfig({
  root: "web",
  base: "./",
  plugins: [react()],
  build: {
    outDir: "../public",
    emptyOutDir: true,
  },
});
