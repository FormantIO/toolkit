import path from "path";
import { defineConfig } from "vite";

export default defineConfig(({ mode }) => {
  const isBundle = mode === "bundle";
  return {
    build: {
      lib: {
        entry: path.resolve(__dirname, "src/main.ts"),
        name: "FormantDataSDK",
        fileName: (format) =>
          `data-sdk${isBundle ? ".bundle" : ""}.${format}.js`,
        formats: ["es", isBundle ? "umd" : "cjs"],
      },
      sourcemap: !isBundle,
      rollupOptions: {
        // make sure to externalize deps that shouldn't be bundled
        // into your library
        external: isBundle ? [] : [/node_modules/],
        output: {
          // Provide global variables to use in the UMD build
          // for externalized deps
          globals: {},
        },
      },
    },
  };
});
