import { Fleet } from "@formant/data-sdk";
import { useEffect, useState } from "react";
import { authenticate } from "../utils/authenticate";
const getStreams = async () => {
  return await authenticate(await Fleet.getStreams());
};

const useStreams = (dependencies: any[] = []) => {
  const [streams, setStreams] = useState([]);

  useEffect(() => {
    getStreams().then((_) => setStreams(_));
  }, dependencies);

  return streams;
};

export default useStreams;
