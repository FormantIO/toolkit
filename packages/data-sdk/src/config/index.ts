import { whichFormantApiUrl } from "./whichFormantApiUrl";

export let FORMANT_API_URL = whichFormantApiUrl(
  typeof window !== "undefined" ? window : globalThis,
  new URLSearchParams(
    typeof window !== "undefined" && window.location
      ? window.location.search
      : undefined
  ),
  typeof window !== "undefined" && window.location
    ? window.location.host
    : undefined
);

export const setFormantApiUrl = (url: string) => {
  FORMANT_API_URL = url;
};
