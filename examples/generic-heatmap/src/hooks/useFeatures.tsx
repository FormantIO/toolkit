import { useEffect, useMemo, useState } from "react";
import { ILocation } from "@formant/data-sdk";
import { generateFeaturesObject } from "../utils/generateFeaturesObject";
import { FeatureCollection } from "geojson";

export const useFeatures = (
  locationDataPoints: ILocation[]
): FeatureCollection => {
  const [features, setFeatures] = useState<FeatureCollection>({
    type: "FeatureCollection",
    features: [],
  });

  useEffect(() => {
    if (locationDataPoints.length < 1) return;
    setFeatures({ ...generateFeaturesObject(locationDataPoints) });
  }, [locationDataPoints]);

  return features;
};
