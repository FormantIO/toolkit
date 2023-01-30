import { useLayoutEffect, useState } from "react";
import { App, ModuleData } from "@formant/data-sdk";

type ScrubberTime = number;

export const useScrubberTime = () => {
  const [time, setTime] = useState<ScrubberTime | null>();
  useLayoutEffect(() => {
    App.addModuleDataListener(_cleanData);
  }, []);

  const _cleanData = (moduleData: ModuleData) => {
    setTime(moduleData.time);
  };
  return time;
};
