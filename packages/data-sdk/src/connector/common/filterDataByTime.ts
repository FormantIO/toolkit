import { IStreamData } from "../../model/IStreamData";

export function filterDataByTime(
  datas: IStreamData[],
  start: Date,
  end: Date
): IStreamData[] {
  const startTime = start.getTime();
  const endTime = end.getTime();
  return datas
    .map((data) => ({
      ...data,
      points: data.points.filter(
        ([timestamp]) => timestamp >= startTime && timestamp < endTime
      ),
    }))
    .filter(({ points }) => points.length > 0);
}
