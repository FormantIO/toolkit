export let FORMANT_API_URL = "https://api.formant.io";

if (typeof window !== "undefined") {
  FORMANT_API_URL = (window as any).FORMANT_API_URL || FORMANT_API_URL;
}
