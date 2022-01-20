export let FORMANT_API_URL = "https://api.formant.io";

if (typeof window !== "undefined") {
  FORMANT_API_URL = (window as any).FORMANT_API_URL || FORMANT_API_URL;
}

let urlParams = new URLSearchParams("");

if (typeof window !== "undefined") {
  urlParams = new URLSearchParams(window.location.search);
}

const moduleName = urlParams.get("formant_dev");
if (moduleName) {
  FORMANT_API_URL = "https://api-dev.formant.io";
}
