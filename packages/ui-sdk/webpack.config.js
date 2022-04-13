const path = require("path");

module.exports = {
  mode: "production",
  entry: "./src/main.ts",
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        use: "ts-loader",
        exclude: /node_modules/,
      },
    ],
  },
  resolve: {
    extensions: [".tsx", ".ts"],
  },
  output: {
    path: path.resolve(__dirname, "dist"),
    filename: "ui-sdk.js",
    library: {
      name: "webpackNumbers",
      type: "umd",
    },
  },
  externals: [
    "lodash",
    "react",
    "@mui/material",
    "@mui/material/styles/ThemeProvider",
    "@mui/material/CssBaseline",
    "@mui/material/styles",
    "@mui/utils",
  ],
};
