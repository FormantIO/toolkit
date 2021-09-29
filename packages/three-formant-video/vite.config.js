const path = require("path");
const { defineConfig } = require("vite");

module.exports = defineConfig({
  build: {
    lib: {
      entry: path.resolve(__dirname, "src/main.ts"),
      name: "Formant3dSDKUrdf",
      fileName: (format) => `three-formant-video.${format}.js`,
    },
    rollupOptions: {
      // make sure to externalize deps that shouldn't be bundled
      // into your library
      external: ["three", "urdf-loader"],
      output: {
        // Provide global variables to use in the UMD build
        // for externalized deps
        globals: {
          three: "ThreeJS",
          "urdf-loader": "UrdfLoader",
        },
      },
    },
  },
});
