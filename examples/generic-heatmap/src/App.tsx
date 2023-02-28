import { useRef, useLayoutEffect } from "react";
import mapboxgl from "mapbox-gl";
import styles from "./App.module.scss";
import { useDataPoints } from "./hooks/useDataPoints";
import { LoadingIndicator, useFormant } from "@formant/ui-sdk";
import { useFeatures } from "./hooks/useFeatures";
import { HeatmapLayer } from "./HeatmapLayer";
import { HeatmapConfiguration } from "./types";
import { DetailsCard } from "./DetailsCard";
import { getTypedConfiguration } from "./utils/utils";

function App() {
  const context = useFormant();
  const config = getTypedConfiguration(
    context.configuration as HeatmapConfiguration
  );
  const dataPoints = useDataPoints();
  const featureCollection = useFeatures(dataPoints);
  console.log(dataPoints);
  const mapContainer = useRef<HTMLDivElement | null>(null);
  const map = useRef<mapboxgl.Map | null>(null);

  useLayoutEffect(() => {
    if (map.current || !config) return; // initialize map only once

    map.current = new mapboxgl.Map({
      accessToken:
        config.mapboxKey ??
        "pk.eyJ1IjoiYWxlbmpkZXYiLCJhIjoiY2t3NWt5ZmExMTcxMDJvbW5kdDR2eGs1diJ9.aYT7nc_i5rp2hY4dt3CLrw",
      container: mapContainer.current!,
      style: "mapbox://styles/alenjdev/ckwcflbv8201814n10hxbqz6q",
      center: [
        config?.longitude ?? -77.47419738769531,
        config?.latitude ?? 39.043701171875,
      ],
      zoom:
        config.defaultZoomLevel && config.latitude && config.longitude
          ? config.defaultZoomLevel
          : 0,
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
            intensity={config.heatmapIntensity}
          />
          <DetailsCard map={map.current} />
        </>
      )}
    </div>
  );
}

export default App;
