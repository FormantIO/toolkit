import { IEventQuery } from "../model/IEventQuery";

import { queryEvents } from "./queryEvents";

export async function getAnnotationCount(query: IEventQuery, tagKey: string) {
  const annotations = await queryEvents({
    ...query,
    eventTypes: ["annotation"],
  });

  const validAnnotations = annotations.filter(
    (annotation) =>
      !!annotation.tags && Object.keys(annotation.tags).includes(tagKey)
  );
  const annotationCounter = validAnnotations.reduce<{
    [key: string]: number;
  }>((prev, current) => {
    const value = current.tags?.[tagKey];
    if (value === undefined) {
      return prev;
    }
    if (value in prev) {
      prev[value] += 1;
      return prev;
    }
    prev[value] = 1;
    return prev;
  }, {});

  return annotationCounter;
}
