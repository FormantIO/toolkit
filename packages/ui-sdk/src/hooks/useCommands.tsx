import { useEffect, useState } from "react";
import { Command, Device } from "@formant/data-sdk";

export const useCommands = (device: Device | undefined): Command[] => {
  const [commands, setCommands] = useState<Command[]>([]);

  useEffect(() => {
    if (!device) return;
    device.getAvailableCommands().then((_) => setCommands(_));
  }, [device]);
  return commands;
};
