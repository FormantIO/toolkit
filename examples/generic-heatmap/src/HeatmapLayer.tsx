import { FC, useLayoutEffect, useState, useRef } from "react";
import { FeatureCollection, Point } from "geojson";
import { GeoJSONSource, LngLatLike } from "mapbox-gl";

interface IHeatmapLayerProps {
  map: mapboxgl.Map | null;
  featureCollection: FeatureCollection<Point>;
  distinctZoomLevel?: number;
  circleRadius?: number;
  intensity?: number;
}

const DEFAULT_DISTINCT_ZOOM_LEVEL = 15;
const DEFAULT_HEATMAP_INTENSITY = 5;

export const HeatmapLayer: FC<IHeatmapLayerProps> = ({
  map,
  featureCollection,
  distinctZoomLevel,
  circleRadius,
  intensity,
}) => {
  const [isLayerLoaded, setIsLayerLoaded] = useState(false);
  const flyingToCounter = useRef(0);
  useLayoutEffect(() => {
    if (!map) return; // wait for map to initialize

    map.on("load", () => {
      map!.addSource("numeric", {
        type: "geojson",
        data: featureCollection,
      });

      map!.addLayer(
        {
          id: "numeric-heat",
          type: "heatmap",
          source: "numeric",
          maxzoom: 24,
          paint: {
            // increase weight as diameter breast height increases
            "heatmap-weight": {
              property: "weight",
              type: "exponential",
              stops: [
                [1, 0],
                [100, 1],
              ],
            },
            // increase intensity as zoom level increases
            "heatmap-intensity": intensity ?? DEFAULT_HEATMAP_INTENSITY,
            // assign color values be applied to points depending on their density
            "heatmap-color": [
              "interpolate",
              ["linear"],
              ["heatmap-density"],
              0,
              "rgba(236,222,239,0)",
              0.2,
              "rgb(169,97,228)",
              0.4,
              "rgb(46,196,149)",
              0.6,
              "rgb(249,195,110)",
              0.8,
              "rgb(234,113,157)", //red
            ],
            // increase radius as zoom increases
            "heatmap-radius": {
              stops: [
                [11, distinctZoomLevel ?? 10],
                [14, distinctZoomLevel ? distinctZoomLevel + 2 : 20],
              ],
            },
            // decrease opacity to transition into the circle layer
            "heatmap-opacity": {
              default: 1,
              stops: [
                [
                  distinctZoomLevel
                    ? distinctZoomLevel - 1
                    : DEFAULT_DISTINCT_ZOOM_LEVEL - 1,
                  1,
                ],
                [distinctZoomLevel ?? DEFAULT_DISTINCT_ZOOM_LEVEL + 1, 0],
              ],
            },
          },
        },
        "waterway-label"
      );
      // add circle layer here
      map!.addLayer(
        {
          id: "numeric-point",
          type: "circle",
          source: "numeric",
          minzoom: !!distinctZoomLevel
            ? distinctZoomLevel - 1
            : DEFAULT_DISTINCT_ZOOM_LEVEL - 1,
          paint: {
            // increase the radius of the circle as the zoom level and weight value increases
            "circle-radius": {
              property: "weight",
              type: "exponential",
              stops: [
                [
                  { zoom: DEFAULT_DISTINCT_ZOOM_LEVEL, value: 1 },
                  circleRadius ?? 5,
                ],
                [{ zoom: 22, value: 100 }, circleRadius ?? 20],
              ],
            },
            "circle-color": {
              property: "weight",
              type: "exponential",
              stops: [
                [0, "rgba(236,222,239,0)"],
                [25, "rgb(169,97,228)"],
                [50, "rgb(46,196,149)"],
                [75, "rgb(249,195,110)"],
                [100, "rgb(234,113,157)"],
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
    setIsLayerLoaded(true);
  }, [map]);

  useLayoutEffect(() => {
    if (!map || featureCollection.features.length < 1 || !isLayerLoaded) return;

    setTimeout(() => {
      const current = map.getSource("numeric") as GeoJSONSource;
      map.addLayer;
      if (!current) return;
      current.setData(featureCollection as any);
      if (flyingToCounter.current === 0) {
        map.flyTo({
          center: featureCollection.features[0].geometry
            .coordinates as LngLatLike,
          zoom: DEFAULT_DISTINCT_ZOOM_LEVEL - 3, // Fly to the selected target
          duration: 5000,
          essential: true,
        });
        flyingToCounter.current = 1;
      }
    }, 3000);
  }, [JSON.stringify(featureCollection.features), isLayerLoaded]);

  return <></>;
};
