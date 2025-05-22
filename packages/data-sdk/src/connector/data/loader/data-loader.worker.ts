import { errorToString } from "../../common/errorToString";

// eslint-disable-next-line @typescript-eslint/no-explicit-any
const context: Worker = self as any;
const previewLength = 1000;

addEventListener("message", async (event) => {
  try {
    const url: string = event.data;

    try {
      const response = await fetch(url, { mode: "cors" });
      const payload = await response.text();

      context.postMessage({
        json: JSON.parse(payload),
        preview: payload.substring(0, previewLength),
        length: payload.length,
        url,
      });
    } catch (error) {
      throw new Error(`Load failed ${errorToString(error)}`);
    }
  } catch (error) {
    context.postMessage({
      error: errorToString(error),
      url: event?.data?.url,
    });
  }
});
