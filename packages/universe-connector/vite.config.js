import path from "node:path";
import { defineConfig } from "vite";

export default defineConfig(({ mode }) => {
  const isBundle = mode === "bundle";

  return {
    build: {
      lib: {
        entry: path.resolve(__dirname, "src/main.ts"),
        name: "FormantUniverseConnector",
        fileName: (format) =>
          `formant-universe-connector${
            isBundle && format !== "umd" ? ".bundle" : ""
          }.${format}.js`,
        formats: ["es", isBundle ? "umd" : "cjs"],
      },
      sourcemap: !isBundle,
      rollupOptions: {
        // make sure to externalize deps that shouldn't be bundled
        // into your library
        external: isBundle
          ? [
              "@formant/data-sdk",
              "@formant/realtime-sdk",
              "@formant/ui-sdk-realtime-player-core",
              "@formant/ui-sdk-realtime-player-core-worker",
              "@formant/universe-core",
            ]
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
              return isNodeModule;
            },
        output: {
          // Provide global variables to use in the UMD build
          // for externalized deps
          globals: isBundle
            ? {
                "@formant/data-sdk": "FormantDataSDK",
                "@formant/realtime-sdk": "FormantRealtimeSDK",
                "@formant/ui-sdk-realtime-player-core":
                  "FormantUISDKRealtimePlayerCore",
                "@formant/ui-sdk-realtime-player-core-worker":
                  "FormantUISDKRealtimePlayerCoreWorker",
                "@formant/universe-core": "FormantUniverseCore",
              }
            : {},
        },
      },
    },
  };
});
