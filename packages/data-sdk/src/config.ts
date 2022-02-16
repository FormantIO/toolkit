export let FORMANT_API_URL = "https://api.formant.io";

if (typeof window !== "undefined") {
  FORMANT_API_URL = (window as any).FORMANT_API_URL || FORMANT_API_URL;
}

let urlParams = new URLSearchParams("");

if (typeof window !== "undefined") {
  urlParams = new URLSearchParams(window.location.search);
}

const envDev = urlParams.get("formant_dev");
if (envDev) {
  FORMANT_API_URL = "https://api-dev.formant.io";
}

const envStage = urlParams.get("formant_stage");
if (envStage) {
  FORMANT_API_URL = "https://api-stage.formant.io";
}
