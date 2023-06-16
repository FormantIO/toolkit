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
        external: isBundle
          ? []
          : (source, importer) => {
              if (!importer) {
                return false;
              }

              const isRelative =
                source.startsWith("./") || source.startsWith("../");

              if (isRelative) {
                return false;
              }

              const isNodeModule = !path.isAbsolute(source);
              if (isNodeModule) {
                return true;
              }

              return false;
            },
        output: {
          // Provide global variables to use in the UMD build
          // for externalized deps
          globals: {},
        },
      },
    },
  };
});
