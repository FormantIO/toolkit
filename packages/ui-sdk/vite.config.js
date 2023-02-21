const path = require("path");
const { defineConfig } = require("vite");

module.exports = defineConfig({
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src/"),
      "@style": `${path.resolve(__dirname, "./src/style")}`,
      "@images": `${path.resolve(__dirname, "./src/images")}`,
    },
  },
  build: {
    lib: {
      entry: path.resolve(__dirname, "src/main.ts"),
      name: "FormantDataSDK",
      fileName: (format) => `ui-sdk.${format}.js`,
    },
    rollupOptions: {
      // make sure to externalize deps that shouldn't be bundled
      // into your library
      external: [
        "@emotion/react",
        "@emotion/styled",
        "@mui/material",
        "@mui/utils",
        "react",
        "react-dom",
        "@mui/material/styles/ThemeProvider",
        "@mui/material/CssBaseline",
        "@mui/material/styles",
        "classnames",
        "styled-components",
        "sass",
      ],
      output: {
        // Provide global variables to use in the UMD build
        // for externalized deps
        globals: {},
      },
    },
  },
});
