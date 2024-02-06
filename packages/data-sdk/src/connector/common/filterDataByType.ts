import { IStreamData } from "../../model/IStreamData";
import { StreamType } from "../../model/StreamType";

export function filterDataByType<T extends StreamType>(
  datas: IStreamData[],
  type: T
): IStreamData<T>[] {
  return datas.filter((_) => _.type === type) as IStreamData<T>[];
}
