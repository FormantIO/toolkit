export function getCurrentModuleContext(): string | null {
  if (!(typeof window !== "undefined" && window.location)) {
    return null;
  }

  const urlParams = new URLSearchParams(window.location.search);
  return urlParams.get("module");
}
