/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_EMAIL: string;
  readonly VITE_PWD: string;
  readonly VITE_DEVICE: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
