export type BrowserType =
  | "Firefox"
  | "Chrome"
  | "Edge"
  | "Safari"
  | "IE"
  | "Other";

export function browser(): BrowserType {
  const { userAgent } = navigator;

  if (userAgent.includes("Firefox/")) {
    return "Firefox";
  } else if (userAgent.includes("Edg/")) {
    return "Edge";
  } else if (userAgent.includes("Chrome/")) {
    return "Chrome";
  } else if (userAgent.includes("Safari/")) {
    return "Safari";
  } else if (userAgent.includes("MSIE/") || userAgent.includes("Trident/")) {
    return "IE";
  } else {
    return "Other";
  }
}
