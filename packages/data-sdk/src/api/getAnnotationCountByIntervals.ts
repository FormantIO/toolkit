import { IEventQuery } from "../model/IEventQuery";
import { AggregateLevel } from "../model/AggregateLevel";
import { aggregateByDateFunctions } from "../utils/aggregateFunctionUtils";
import { getAnnotationCount } from "./getAnnotationCount";

export async function getAnnotationCountByIntervals(
  query: IEventQuery,
  tagKey: string,
  aggregate: AggregateLevel
) {
  const { end, start } = query;
  const dateFunctions = aggregateByDateFunctions[aggregate];
  const intervals: Date[] = dateFunctions.interval({
    start: new Date(start!),
    end: new Date(end!),
  });

  const annotationsQuery = intervals.map((_, idx) => {
    const startDate = new Date(_).toISOString();
    const endDate =
      idx === intervals.length - 1
        ? new Date(Date.now()).toISOString()
        : new Date(intervals[idx + 1]);
    return getAnnotationCount(
      {
        ...query,
        start: startDate,
        end: endDate as string,
      },
      tagKey
    );
  });
  const responses = await Promise.all(annotationsQuery);

  return intervals.map((_, idx) => ({
    date: new Date(_).toISOString(),
    annotations: responses[idx],
  }));
}
