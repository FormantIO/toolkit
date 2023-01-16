import { ILocation } from "@formant/data-sdk";
import { FeatureCollection } from "geojson";

export const generateFeaturesObject = (
  locationDataPoints: ILocation[]
): FeatureCollection => {
  return {
    type: "FeatureCollection",
    features: locationDataPoints.map((d) => ({
      type: "Feature",
      properties: {
        weight: 40,
      },
      geometry: {
        type: "Point",
        coordinates: [d.latitude, d.longitude],
      },
    })),
  };
};
