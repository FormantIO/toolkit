import { describe, it, expect } from "vitest";
import { formatTimeFrameText } from "./aggregateFunctionUtils";

const start = new Date(2026, 7, 24);
const end = new Date(2026, 7, 31);

describe("formatTimeFrameText()", () => {
  describe("dates", () => {
    it.each<[string, string]>([
      ["en-US", "8/24–8/31"],
      ["en-GB", "24/08–31/08"],
      ["ja-JP", "8/24–8/31"],
      ["bg-BG", "24.08–31.08"],
    ])("should drop the year for %s", (locale, expected) => {
      expect(formatTimeFrameText(start, end, locale)).toBe(expected);
    });

    it("should accept timestamps", () => {
      expect(formatTimeFrameText(start.getTime(), end.getTime(), "en-US")).toBe(
        "8/24–8/31"
      );
    });
  });

  describe("localized date strings", () => {
    it.each<[string, string]>([
      ["en-US", "8/24–8/31"],
      ["en-GB", "24/08–31/08"],
      ["ja-JP", "8/24–8/31"],
      ["bg-BG", "24.08–31.08"],
    ])("should drop the year for %s", (locale, expected) => {
      expect(
        formatTimeFrameText(
          start.toLocaleDateString(locale),
          end.toLocaleDateString(locale),
          locale
        )
      ).toBe(expected);
    });
  });

  describe("every locale", () => {
    const locales = [
      "en-US",
      "en-GB",
      "ja-JP",
      "bg-BG",
      "de-DE",
      "fr-FR",
      "ko-KR",
      "hu-HU",
      "ar-EG",
      "hi-IN",
    ];

    it.each(locales)("should keep both days for %s", (locale) => {
      const digits = new Intl.NumberFormat(locale, { useGrouping: false });
      const text = formatTimeFrameText(start, end, locale);

      expect(text).toContain(digits.format(start.getDate()));
      expect(text).toContain(digits.format(end.getDate()));
      expect(text).not.toContain("undefined");
    });

    it.each(locales)(
      "should format a localized string like the date it came from for %s",
      (locale) => {
        expect(
          formatTimeFrameText(
            start.toLocaleDateString(locale),
            end.toLocaleDateString(locale),
            locale
          )
        ).toBe(formatTimeFrameText(start, end, locale));
      }
    );
  });

  describe("unusable input", () => {
    it.each<[string, string, string]>([
      ["text", "last tuesday", "last tuesday–next tuesday"],
      ["an empty string", "", "–"],
      ["an impossible date", "2/30/2026", "2/30/2026–2/31/2026"],
    ])("should echo back %s", (_, unusable, expected) => {
      const unusableEnd = unusable.replace("last", "next").replace("30", "31");

      expect(formatTimeFrameText(unusable, unusableEnd, "en-US")).toBe(
        expected
      );
    });

    it("should render an invalid date as an empty boundary", () => {
      expect(formatTimeFrameText(new Date(NaN), end, "en-US")).toBe("–8/31");
    });
  });
});
