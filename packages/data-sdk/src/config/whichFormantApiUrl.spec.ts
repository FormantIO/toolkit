import { describe, it, expect } from "vitest";
import { whichFormantApiUrl } from "./whichFormantApiUrl";

describe("whichFormantApiUrl()", () => {
  it("should return the default url", () => {
    const url = whichFormantApiUrl(undefined, undefined);
    expect(url).toBe("https://api.formant.io");
  });

  describe("defined environments", () => {
    it.each<[string, string]>([
      ["formant_dev", "https://api-dev.formant.io"],
      ["formant_stage", "https://api-stage.formant.io"],
      ["formant_local", "https://api.formant.local"],
    ])("should", (urlKey, expected) => {
      const params = new URLSearchParams(`${urlKey}`);
      const url = whichFormantApiUrl(undefined, params);
      expect(url).toBe(expected);
    });
  });

  it(`should look up a setting on the "global" object`, () => {
    const global = { FORMANT_API_URL: "https://custom.formant.local" };
    const url = whichFormantApiUrl(global, undefined);
    expect(url).toBe("https://custom.formant.local");
  });

  it(`should use pre-defined environments over glbal`, () => {
    const params = new URLSearchParams(`foo&formant_stage&bar&baz`);
    const global = { FORMANT_API_URL: "https://custom.formant.local" };
    const url = whichFormantApiUrl(global, params);
    expect(url).toBe("https://api-stage.formant.io");
  });
});
