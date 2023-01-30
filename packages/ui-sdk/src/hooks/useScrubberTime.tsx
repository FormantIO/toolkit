import { App, ModuleData } from "@formant/data-sdk";
import * as React from "react";
import { authenticate } from "../utils/authenticate";

const getScrubberTime = async (setter: any) => {
  await authenticate(async () => {
    App.addModuleDataListener((_: ModuleData) => setter(_.time));
  });
};

export const useScruberTime = () => {
  const [time, setTime] = React.useState();

  React.useEffect(() => {
    getScrubberTime(setTime);
  }, []);

  return time;
};

export default useScruberTime;
