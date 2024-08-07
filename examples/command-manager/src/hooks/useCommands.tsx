import { useEffect, useState } from "react";
import { Authentication, Device, Command } from "@formant/data-sdk";
import { useDevice } from "@formant/ui-sdk";

const getCommands = async (device: Device) => {
  if (await Authentication.waitTilAuthenticated()) {
    const commands = await device.getAvailableCommands();
    return commands.filter((_: any) => _.enabled);
  }
};

export const useCommands = () => {
  const device = useDevice();
  const [commands, setCommands] = useState<Command[] | null>(null);

  useEffect(() => {
    if (!device) return;
    getCommands(device as any).then((_) => {
      if (!_) return;
      setCommands(_);
    });
  }, [device]);

  return commands;
};
