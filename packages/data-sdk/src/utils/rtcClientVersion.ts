let version: string | null | undefined;

export function overrideRtcClientVersion(newVersion: string | null) {
  version = newVersion;
}

export function getRtcClientVersion(): string | null {
  if (version === undefined) {
    // get query param for "rtc_client"
    const urlParams = new URLSearchParams(window.location.search);
    version = urlParams.get("rtc_client");
  }
  return version;
}
