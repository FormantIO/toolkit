import { Authentication, Fleet, IEvent } from "@formant/data-sdk";
import { useEffect, useState } from "react";
import { useDevice } from "@formant/ui-sdk";

const getEvents = async (
  eventName: string,
  deviceId: string
): Promise<IEvent[] | undefined> => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const events = await Fleet.queryEvents({
        names: [eventName],
        deviceIds: [deviceId],
      });
      return events;
    }
  } catch (error) {
    throw error;
  }
};

export const useEvents = (eventName: string) => {
  const device = useDevice();
  const [events, setEvents] = useState<IEvent[]>([]);

  useEffect(() => {
    if (!device) return;
    getEvents(eventName, device.id).then((_) => setEvents(_));
  }, [device]);
  return events;
};
