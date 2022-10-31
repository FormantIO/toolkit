const _baseUrl = () => {
    let _ = "https://api.formant.io/v1/admin";
    const urlParams = new URLSearchParams(window.location.search);
    const dev = urlParams.get("formant_dev") === "true";
    const stage = urlParams.get("formant_stage") === "true";
    if (dev) {
      _ = "https://api-dev.formant.io/v1/admin";
    }
    if (stage) {
      _ = "https://api-stage.formant.io/v1/admin";
    }
    return _;
  };
  
  export const baseUrl = _baseUrl();
  