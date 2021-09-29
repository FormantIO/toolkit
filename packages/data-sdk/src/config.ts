export let FORMANT_API_URL = "https://api.formant.io";

if (window) {
  FORMANT_API_URL = (window as any).FORMANT_API_URL || FORMANT_API_URL;
}
