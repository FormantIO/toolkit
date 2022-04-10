import { Button } from "@alenjdev/ui-sdk";
import { App, Device, ModuleData } from "@formant/data-sdk";
import React, { useEffect, FC } from "react";
import { useState } from "react";
import loading from "../images/loading.png";

interface IDownloadBtnProps {
  device: Device | undefined;
}

interface IVideoStream {
  duration: number;
  mimeType: "video/mp4";
  size: number;
  url: string;
}

export const DownloadBtn: FC<IDownloadBtnProps> = ({ device }) => {
  const [url, setUrl] = useState<undefined | string>(undefined);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    App.addModuleDataListener(getLatestData);
  }, []);

  const getLatestData = async (newData: ModuleData) => {
    try {
      const video = getLatestJsonUrl(newData);
      if (!video) return;
      setUrl(video.url);
    } catch (error) {
      setUrl(undefined);
    }
  };

  const downloadVideo = async () => {
    setIsLoading(true);
    const response = await fetch(url!);
    const blob = await response.blob();
    const urls = window.URL.createObjectURL(new Blob([blob]));
    const link = document.createElement("a");
    link.href = urls;
    link.setAttribute("download", `${device?.name}_video.mp4`);
    document.body.appendChild(link);
    link.click();
    link.parentNode!.removeChild(link);
    setIsLoading(false);
  };

  return (
    <React.Fragment>
      {isLoading ? (
        <img height={30} src={loading} />
      ) : (
        <Button
          disabled={url === undefined}
          onClick={downloadVideo}
          type="primary"
          size="medium"
        >
          Download
        </Button>
      )}
    </React.Fragment>
  );
};

function getLatestJsonUrl(moduleData: ModuleData): IVideoStream | undefined {
  const streams = Object.values(moduleData.streams);
  if (streams.length === 0) {
    throw new Error("No streams.");
  }
  const stream = streams[0];
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
  return latestPoint[1];
}

export default App;
