import { useEffect, useState } from "react";
import { App, ModuleData } from "@formant/data-sdk";

interface data {
  agentId: string;
  deviceId: string;
  name: string;
  tags: { [key: string]: string };
  points: [number, any][];
  type: string;
}

type IStream = [string, Stream];

interface Stream {
  loading: boolean;
  tooMuchData: boolean;
  data: data[];
  type: string;
}

type ScrubberTime = { time: number };

//Intended to be use for custom modules within Formant

export const useScrubberTime = () => {
  const [streams, setStreams] = useState<ScrubberTime | null>();
  useEffect(() => {
    App.addModuleDataListener(_cleanData);
  }, []);

  const _cleanData = (moduleData: ModuleData) => {
    let streams: any = Object.entries(moduleData.streams);

    //check is is there any stream coming in
    if (streams.length === 0) {
      setStreams(null);
    }

    const _cleanStreams = streams.reduce((_: any, stream: IStream) => {
      const streamName = stream[0];
      const streamValue = stream[1];
      if (streamValue === undefined) {
        throw new Error("No stream.");
      }
      if (streamValue.loading) {
        return _;
      }
      if (streamValue.tooMuchData) {
        return _;
      }
      if (streamValue.data.length === 0) {
        _[streamName] = null;
        return _;
      }
      const latestPoint = streamValue.data[0].points.at(-1);

      if (!latestPoint) {
        _[streamName] = null;
        return _;
      }
      _.time = latestPoint[0];

      return _;
    }, {});
    if (Object.keys(_cleanStreams).length === 0) return;
    setStreams(_cleanStreams);
  };
  //Returns an object with time ex: {time: 1672853001358}
  return streams;
};
