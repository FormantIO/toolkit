import { useRef, useLayoutEffect } from "react";
import mapboxgl from "mapbox-gl";
import styles from "./App.module.scss";
import { useDataPoints } from "./hooks/useDataPoints";
import { LoadingIndicator, useFormant } from "@formant/ui-sdk";
import { useFeatures } from "./hooks/useFeatures";
import { HeatmapLayer } from "./HeatmapLayer";
import { IConfiguration } from "./types";
import { DetailsCard } from "./DetailsCard";

mapboxgl.accessToken =
  "pk.eyJ1IjoiYWxlbmpkZXYiLCJhIjoiY2t3NWt5ZmExMTcxMDJvbW5kdDR2eGs1diJ9.aYT7nc_i5rp2hY4dt3CLrw";

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const dataPoints = useDataPoints();
  const featureCollection = useFeatures(dataPoints);
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
            distinctZoomLevel={config.distinctZoomLevel}
            circleRadius={config.circleRadius}
          />
          <DetailsCard map={map.current} />
        </>
      )}
    </div>
  );
}

export default App;