import { parse, loadFromBase64 } from "./pcd";

onmessage = (
  event: MessageEvent<{ id?: number; url?: string; pointCloud: string }>
) => {
  const { url, id, pointCloud } = event.data;
  (async () => {
    if (url) {
      const pcd = parse(
        await fetch(url, { mode: "cors" }).then((r) => r.arrayBuffer())
      );
      postMessage({ url, pcd });
    }
    if (id && pointCloud) {
      const pcd = loadFromBase64(pointCloud);
      postMessage({ id, pcd });
    }
  })();
};
