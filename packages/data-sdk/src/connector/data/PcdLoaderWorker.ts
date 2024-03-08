import { duration } from "../common/duration";
import { StoreCache } from "./StoreCache";
import { parse, loadFromBase64, IPcd } from "./pcd";

const cache = new StoreCache<string, IPcd>({
  capacity: 1000,
  timeout: 1 * duration.minute,
});

onmessage = async (
  event: MessageEvent<{ id?: number; url?: string; pointCloud: string }>
) => {
  const { url, id, pointCloud } = event.data;

  if (url) {
    // Check if the point cloud for the given URL is already in the cache
    const cached = cache.get(url, async () => {
      // Fetch the point cloud and store it in the cache
      const pcd = parse(
        await fetch(url, { mode: "cors" }).then((r) => r.arrayBuffer())
      );
      return pcd;
    });

    if (cached) {
      postMessage({ url, pcd: cached });
    }
  }

  if (id && pointCloud) {
    // Process the base64-encoded point cloud
    const pcd = loadFromBase64(pointCloud);
    postMessage({ id, pcd });
  }
};
