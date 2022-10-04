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

interface Stream {
  loading: boolean;
  tooMuchData: boolean;
  data: data[];
  type: string;
}

type Streams = { [key: string]: any };

//Intended to be use for custom modules within Formant

const useModuleDataListener = () => {
  const [streams, setStreams] = useState<Streams>({});
  useEffect(() => {
    App.addModuleDataListener(_cleanData);
  }, []);

  const _cleanData = (moduleData: ModuleData) => {
    let streams: any = Object.values(moduleData.streams);
    //check is is there any stream coming in
    if (streams.length === 0) {
      throw new Error("No streams.");
    }
    setStreams(
      streams.reduce((_: any, stream: Stream) => {
        if (stream === undefined) {
          throw new Error("No stream.");
        }
        if (stream.loading) {
          return undefined;
        }
        if (stream.tooMuchData) {
          throw new Error("Too much data.");
        }
        if (stream.data.length === 0) {
          throw new Error("No data.");
        }
        const latestPoint = stream.data[0].points.at(-1);

        if (!latestPoint) {
          throw new Error("No datapoints.");
        }

        return { ..._, [stream.data[0].name]: latestPoint[1] };
      }, {})
    );
  };
  //Returns an object with stream name as key and last know value
  return streams;
};

export default useModuleDataListener;
