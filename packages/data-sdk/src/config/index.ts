import { whichFormantApiUrl } from "./whichFormantApiUrl";

export const FORMANT_API_URL = whichFormantApiUrl(
  window,
  typeof window !== "undefined"
    ? new URLSearchParams(window.location.search)
    : undefined
);
