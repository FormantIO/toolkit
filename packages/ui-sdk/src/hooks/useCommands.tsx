import { useEffect, useState } from "react";
import { Command, Device } from "@formant/data-sdk";
import { authenticate } from "../utils/authenticate";

const getCommands = async (device: Device) =>
  await authenticate(await device.getAvailableCommands());

const useCommands = (device: Device | undefined): Command[] => {
  const [commands, setCommands] = useState<Command[]>([]);

  useEffect(() => {
    if (!device) return;
    getCommands(device).then((_) => setCommands(_));
  }, [device]);
  return commands;
};

export default useCommands;
