import { EventType } from "../model/EventType";
import { AggregateLevel } from "../model/AggregateLevel";
import { IEventQuery } from "../model/IEventQuery";
import {
  aggregateByDateFunctions,
  formatTimeFrameText,
} from "../utils/aggregateFunctionUtils";
import { queryEvents } from "./queryEvents";

export async function eventsCounter(
  eventTypes: EventType[],
  timeFrame: AggregateLevel,
  range: number,
  time: number,
  query?: IEventQuery
) {
  const dateFunctions = aggregateByDateFunctions[timeFrame];

  return await Promise.all(
    Array(range)
      .fill(0)
      .map(async (_, dateOffset) => {
        const activePointInTimeLine = new Date(time);

        const startDate: Date = dateFunctions.sub(
          dateFunctions.start(activePointInTimeLine),
          range - dateOffset - 1
        );
        const endDate: Date = dateFunctions.sub(
          dateFunctions.end(activePointInTimeLine),
          range - dateOffset - 1
        );
        const date = formatTimeFrameText(
          startDate.toLocaleDateString(),
          endDate.toLocaleDateString()
        );
        const events = await queryEvents({
          ...query,
          eventTypes,
          start: new Date(startDate).toISOString(),
          end: new Date(endDate).toISOString(),
        });
        return { date, events };
      })
  );
}
