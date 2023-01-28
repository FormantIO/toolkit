import { useEffect, useState } from "react";
import { generateFeaturesObject } from "../utils/utils";
import { FeatureCollection, Point } from "geojson";
import { IHeatMapDataPoint } from "../types";

export const useFeatures = (
  locationDataPoints: IHeatMapDataPoint[]
): FeatureCollection<Point> => {
  const [features, setFeatures] = useState<FeatureCollection<Point>>({
    type: "FeatureCollection",
    features: [],
  });

  useEffect(() => {
    if (locationDataPoints.length < 1) return;
    setFeatures({ ...generateFeaturesObject(locationDataPoints) });
  }, [locationDataPoints]);

  return features;
};
