import { useRef, useLayoutEffect } from "react";
import mapboxgl, { GeoJSONSource } from "mapbox-gl";
import styles from "./App.module.scss";
import { useConfiguration } from "./hooks/useConfiguration";
import { useLocationDataPoints } from "./hooks/useLocationDataPoints";
import { useDevice } from "@formant/ui-sdk";
import { useFeatures } from "./hooks/useFeatures";
import { HeatmapLayer } from "./HeatmapLayer";
import { LoadingIndicator } from "@formant/ui-sdk";

mapboxgl.accessToken =
  "pk.eyJ1IjoiYWxlbmpkZXYiLCJhIjoiY2t3NWt5ZmExMTcxMDJvbW5kdDR2eGs1diJ9.aYT7nc_i5rp2hY4dt3CLrw";

function App() {
  const device = useDevice();
  const config = useConfiguration();
  const locationDataPoints = useLocationDataPoints(device as any, config!);
  const featureCollection = useFeatures(locationDataPoints);
  const mapContainer = useRef<HTMLDivElement | null>(null);
  const map = useRef<mapboxgl.Map | null>(null);

  useLayoutEffect(() => {
    if (map.current || !config) return; // initialize map only once

    map.current = new mapboxgl.Map({
      container: mapContainer.current!,
      style: "mapbox://styles/alenjdev/ckwcflbv8201814n10hxbqz6q",
      center: [
        config?.longitude ?? -77.47419738769531,
        config?.latitude ?? 39.043701171875,
      ],
      zoom: config?.zoom ?? 0,
    });
  }, [config]);

  return (
    <div className={styles.app}>
      {!config ? (
        <LoadingIndicator />
      ) : (
        <>
          <div ref={mapContainer} className={styles["map-container"]} />
          <HeatmapLayer
            map={map.current}
            featureCollection={featureCollection}
          />
        </>
      )}
    </div>
  );
}

export default App;
