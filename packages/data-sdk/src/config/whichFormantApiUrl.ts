const DEFAULT_FORMANT_API_URL = "https://api.formant.io";

interface IHasString {
  has(key: string): boolean;
}

export function whichFormantApiUrl(
  global: any,
  urlParams: IHasString | undefined
) {
  if (urlParams) {
    if (urlParams.has("formant_stage")) {
      return "https://api-stage.formant.io";
    }

    if (urlParams.has("formant_dev")) {
      return "https://api-dev.formant.io";
    }

    if (urlParams.has("formant_local")) {
      return "https://api.formant.local";
    }
  }

  if (
    typeof global !== "undefined" &&
    "FORMANT_API_URL" in global &&
    typeof global.FORMANT_API_URL === "string"
  ) {
    return global.FORMANT_API_URL;
  }

  return DEFAULT_FORMANT_API_URL;
}
