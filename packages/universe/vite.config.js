const path = require("path");
const { defineConfig } = require("vite");

module.exports = defineConfig({
  build: {
    lib: {
      entry: path.resolve(__dirname, "src/main.ts"),
      name: "FormantDataSDK",
      fileName: (format) => `universe.${format}.js`,
    },
    rollupOptions: {
      // make sure to externalize deps that shouldn't be bundled
      // into your library
      external: [
        "three",
        "react",
        "@formant/ui-sdk",
        "react-use-measure",
        "styled-components",
        "geolib",
        "uuid",
        "jszip",
        "lzfjs",
        "urdf-loader",
        "@formant/ui-sdk-realtime-player-core",
        "recoil",
        "recoil-nexus",
        "immer",
      ],
      output: {
        // Provide global variables to use in the UMD build
        // for externalized deps
        globals: {},
      },
    },
  },
});
