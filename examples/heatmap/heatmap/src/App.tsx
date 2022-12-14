import { useRef, useEffect, useLayoutEffect, useState } from "react";
import mapboxgl, { GeoJSONSource } from "mapbox-gl";
import styles from "./App.module.scss";
import { App, ModuleData } from "@formant/data-sdk";
import { DeviceCard } from "./DeviceCard";

interface feature {
  type: "Feature";
  properties: {
    weight: number;
  };
  geometry: {
    type: "Point";
    coordinates: any[];
  };
}

mapboxgl.accessToken =
  "pk.eyJ1IjoiYWxlbmpkZXYiLCJhIjoiY2t3NWt5ZmExMTcxMDJvbW5kdDR2eGs1diJ9.aYT7nc_i5rp2hY4dt3CLrw";

export default function Apps() {
  const mapContainer = useRef<HTMLDivElement | null>(null);
  const map = useRef<mapboxgl.Map | null>(null);
  const [pointLatitude, setPointLatitude] = useState<number | string>();
  const [pointLongitude, setPointLongitude] = useState<number | string>();
  const [pointWeight, setPointWeight] = useState<number | string>();

  const [point, setPoint] = useState<feature[]>([
    {
      type: "Feature",
      properties: {
        weight: 10,
      },
      geometry: {
        type: "Point",
        coordinates: [],
      },
    },
  ]);

  const heatMapGeoJSON = () => {
    //Returns and geoJSON type file
    let updateHeatmap = {
      type: "FeatureCollection",
      features: point,
    };

    return updateHeatmap;
  };

  useEffect(() => {
    App.addModuleDataListener(receiveModuleData);
  }, []);

  const receiveModuleData = async (newValue: ModuleData) => {
    const streams = newValue.streams;
    if (Object.keys(streams).length === 0) {
      throw new Error("No streams.");
    }
    let latitude: number, longitude: number, weight: number;

    Object.keys(streams).forEach((stream) => {
      const latestState = getLatestData(streams, stream);
      if (typeof latestState !== "string" && latestState !== undefined) {
        if (streams[stream].data[0].name === "$.host.geoip") {
          latitude = latestState[1].latitude;
          longitude = latestState[1].longitude;
        }
        if (streams[stream].data[0].name === "heatmap_point_weight") {
          weight = latestState[1];
        }
      }
      weight = 1;
      if (!!latitude && !!longitude && !!weight) {
        let feature: feature = {
          type: "Feature",
          properties: {
            weight: weight,
          },
          geometry: {
            type: "Point",
            coordinates: [longitude, latitude],
          },
        };

        setPoint((point) => [...point, feature]);
      }
    });
  };

  useLayoutEffect(() => {
    if (map.current) return; // initialize map only once
    map.current = new mapboxgl.Map({
      container: mapContainer.current!,
      style: "mapbox://styles/alenjdev/ckwcflbv8201814n10hxbqz6q",
      center: [-77.47419738769531, 39.043701171875],
      zoom: 18,
    });
  }, []);

  useLayoutEffect(() => {
    if (!map.current) return; // wait for map to initialize

    const heatMap = heatMapGeoJSON();

    map.current.on("load", () => {
      map.current!.addSource("numeric-example", {
        type: "geojson",
        data: heatMap as any,
      });

      map.current!.addLayer(
        {
          id: "numeric-example-heat",
          type: "heatmap",
          source: "numeric-example",
          maxzoom: 15,
          paint: {
            // increase weight as diameter breast height increases
            "heatmap-weight": {
              property: "weight",
              type: "exponential",
              stops: [[1, 16]],
            },
            // increase intensity as zoom level increases
            "heatmap-intensity": {
              stops: [
                [11, 1],
                [15, 3],
              ],
            },
            // assign color values be applied to points depending on their density
            "heatmap-color": [
              "interpolate",
              ["linear"],
              ["heatmap-density"],
              0,
              "rgba(236,222,239,0)",
              0.2,
              "rgb(208,209,230)",
              0.4,
              "rgb(166,189,219)",
              0.6,
              "rgb(103,169,207)",
              0.8,
              "rgb(155, 21, 0)",
            ],
            // increase radius as zoom increases
            "heatmap-radius": {
              stops: [
                [11, 15],
                [15, 20],
              ],
            },
            // decrease opacity to transition into the circle layer
            "heatmap-opacity": {
              default: 1,
              stops: [
                [14, 1],
                [15, 0],
              ],
            },
          },
        },
        "waterway-label"
      );
      // add circle layer here
      map.current!.addLayer(
        {
          id: "numeric-example-point",
          type: "circle",
          source: "numeric-example",
          minzoom: 14,
          paint: {
            // increase the radius of the circle as the zoom level and weight value increases
            "circle-radius": {
              property: "weight",
              type: "exponential",
              stops: [
                [{ zoom: 15, value: 1 }, 5],
                [{ zoom: 15, value: 62 }, 10],
                [{ zoom: 22, value: 1 }, 20],
                [{ zoom: 22, value: 62 }, 50],
              ],
            },
            "circle-color": {
              property: "weight",
              type: "exponential",
              stops: [
                [0, "rgba(236,222,239,0)"],
                [10, "rgb(236,222,239)"],
                [20, "rgb(208,209,230)"],
                [30, "rgb(166,189,219)"],
                [40, "rgb(103,169,207)"],
                [50, "rgb(28,144,153)"],
                [60, "rgb(1,108,89)"],
              ],
            },
            "circle-stroke-color": "white",
            "circle-stroke-width": 1,
            "circle-opacity": {
              stops: [
                [14, 0],
                [15, 1],
              ],
            },
          },
        },
        "waterway-label"
      );
    });
  }, [map.current]);

  useLayoutEffect(() => {
    if (!map.current) return; // wait for map to initialize

    const card = document.getElementById("card");

    map.current.on("mouseover", "numeric-example-point", (event) => {
      const { x, y } = event.point;
      const { weight } = event.features![0].properties!;
      const { lat, lng } = event.lngLat;

      let ev_lat = lat.toString();
      let ev_lng = lng.toString();

      setPointWeight(weight);
      setPointLatitude(ev_lat.slice(0, 8));
      setPointLongitude(ev_lng.slice(0, 9));

      card!.classList.add("fade-in");
      card!.classList.remove("fade-out");
      card!.style.top = `${y}px`;
      card!.style.left = `${x}px`;
    });
    map.current.on("mouseleave", "numeric-example-point", (event) => {
      card!.classList.remove("fade-in");
      card!.classList.add("fade-out");
    });
  }, []);

  useLayoutEffect(() => {
    if (!map.current) return;
    const heatmap = heatMapGeoJSON();
    if (heatmap.features.length === 1) return;
    let current = map.current.getSource("numeric-example") as GeoJSONSource;

    if (!current) return;
    current.setData(heatmap as any);
  }, [point]);

  return (
    <div className={styles.app}>
      <div ref={mapContainer} className={styles["map-container"]} />
      <DeviceCard
        latitude={pointLatitude}
        longitude={pointLongitude}
        weight={pointWeight}
      />
    </div>
  );
}

const getLatestData = (
  moduleData: any,
  stream: string
): any | string | undefined => {
  if (moduleData[stream] === undefined) {
    return "No stream.";
  }
  if (moduleData[stream].loading) {
    return undefined;
  }
  if (moduleData[stream].tooMuchData) {
    return "Too much data.";
  }

  if (moduleData[stream].data.length === 0) {
    return "No data.";
  }
  const latestPoint = moduleData[stream].data[0].points.at(-1);
  if (!latestPoint) {
    return "No datapoints.";
  }
  return latestPoint;
};
