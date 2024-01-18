import path from "path";
import { defineConfig } from "vite";

export default defineConfig(({ mode }) => {
  const isBundle = mode === "bundle";
  return {
    define: {
      "process.env.NODE_ENV": JSON.stringify("production"),
    },
    build: {
      lib: {
        entry: path.resolve(__dirname, "src/main.ts"),
        name: "FormantDataSDK",
        fileName: (format) =>
          `data-sdk.${isBundle && format === "es" ? "es6" : format}.js`,
        formats: ["es", isBundle ? "umd" : "cjs"],
      },
      sourcemap: !isBundle,
      rollupOptions: {
        // make sure to externalize deps that shouldn't be bundled
        // into your library
        external: [],
        output: {
          // Provide global variables to use in the UMD build
          // for externalized deps
          globals: {},
        },
      },
    },
  };
});
