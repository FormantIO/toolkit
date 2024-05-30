import { duration } from "../common/duration";
import { StoreCache } from "./StoreCache";

const cache = new StoreCache<string, any>({
  capacity: 1000,
  timeout: 1 * duration.minute,
});

onmessage = async (event: MessageEvent<{ url?: string }>) => {
  const { url } = event.data;

  if (url) {
    // Check if the point cloud for the given URL is already in the cache
    const cached = cache.get(url, async () => {
      // Fetch the point cloud and store it in the cache
      const res = await fetch(url, { mode: "cors" }).then((r) => r.json());
      return res;
    });

    if (cached) {
      postMessage({ url, response: cached });
    }
  }
};
