import { Authentication } from "@formant/data-sdk";
import { useEffect, useState } from "react";

const getStreams = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    const response = await fetch(
      "https://api.formant.io/v1/admin/streams",
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    const streams = await response.json();
    return streams.items.filter((_: { enabled: boolean }) => _.enabled);
  }
};

export const useStreams = (device: any) => {
  const [streams, setStreams] = useState([]);

  useEffect(() => {
    if (!device) return;
    getStreams().then((_) => setStreams(_));
  }, [device]);

  return streams;
};
