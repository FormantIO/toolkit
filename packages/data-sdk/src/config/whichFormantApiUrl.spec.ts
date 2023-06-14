import { describe, it, expect } from "vitest";
import { whichFormantApiUrl } from "./whichFormantApiUrl";

describe("whichFormantApiUrl()", () => {
  it("should return the default url", () => {
    const url = whichFormantApiUrl({}, new URLSearchParams());
    expect(url).toBe("https://api.formant.io");
  });

  describe("defined environments", () => {
    it.each<[string, string]>([
      ["formant_dev", "https://api-dev.formant.io"],
      ["formant_stage", "https://api-stage.formant.io"],
      ["formant_local", "https://api.formant.local"],
    ])("should", (urlKey, expected) => {
      const params = new URLSearchParams(`${urlKey}=true`);
      const url = whichFormantApiUrl({}, params);
      expect(url).toBe(expected);
    });
  });

  it(`should look up a setting on the "global" object`, () => {
    const global = { FORMANT_API_URL: "https://custom.formant.local" };
    const url = whichFormantApiUrl(global, new URLSearchParams());
    expect(url).toBe("https://custom.formant.local");
  });

  describe("custom urls", () => {
    it("should accept custom API urls in query string", () => {
      const params = new URLSearchParams(
        `foo&formant_url=${encodeURIComponent(
          "https://some.other.formant.domain.local"
        )}&bar&baz`
      );
      const url = whichFormantApiUrl({}, params);
      expect(url).toBe("https://some.other.formant.domain.local");
    });

    it("should ignore custom API invalid urls", () => {
      const params = new URLSearchParams(
        `foo&formant_url=${encodeURIComponent("this-isn't-a-url")}&bar&baz`
      );
      const url = whichFormantApiUrl({}, params);
      expect(url).toBe("https://api.formant.io");
    });
  });

  describe("precedence", () => {
    it(`should use pre-defined environments over custom and global`, () => {
      const params = new URLSearchParams(
        `foo&formant_stage=true&formant_url=wat&baz`
      );
      const global = { FORMANT_API_URL: "https://custom.formant.local" };
      const url = whichFormantApiUrl(global, params);
      expect(url).toBe("https://api-stage.formant.io");
    });

    it(`should use custom-defined environments over global`, () => {
      const params = new URLSearchParams(
        `foo&formant_url=${encodeURIComponent(
          "https://some.other.formant.domain.local"
        )}&bar&baz`
      );
      const global = { FORMANT_API_URL: "https://custom.formant.local" };
      const url = whichFormantApiUrl(global, params);
      expect(url).toBe("https://some.other.formant.domain.local");
    });
  });
});
