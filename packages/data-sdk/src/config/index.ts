import { whichFormantApiUrl } from "./whichFormantApiUrl";

export const FORMANT_API_URL = whichFormantApiUrl(
  window,
  new URLSearchParams(
    (typeof window !== "undefined" && window.location) ? window.location.search : undefined
  )
);
