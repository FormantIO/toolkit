import path from "path";
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

module.exports = defineConfig(({ mode }) => {
  const isBundle = mode === "bundle";
  return {
    define: {
      "process.env.NODE_ENV": JSON.stringify(
        process.env.NODE_ENV ?? "production"
      ),
    },
    plugins: [
      react({
        jsxImportSource: "@emotion/react",
        babel: {
          plugins: ["@emotion/babel-plugin"],
        },
      }),
    ],
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
        name: "FormantUISDk",
        fileName: (format) =>
          `ui-sdk${isBundle && format !== "umd" ? ".bundle" : ""}.${format}.js`,
        formats: ["es", isBundle ? "umd" : "cjs"],
      },
      sourcemap: !isBundle,
      rollupOptions: {
        external: isBundle
          ? ["@formant/ui-sdk"]
          : (source, importer) => {
              if (!importer) return false;

              const isRelative =
                source.startsWith("./") || source.startsWith("../");

              if (isRelative) {
                const abs = path.resolve(importer, source);
                return !abs.startsWith(path.resolve(__dirname, "./src/"));
              }

              if (!path.isAbsolute(source)) {
                return true;
              }

              return !source.startsWith(path.resolve(__dirname, "./src/"));
            },
        output: {
          globals: isBundle ? { "@formant/ui-sdk": "FormantDataSDK" } : {},
        },
      },
    },
  };
});
