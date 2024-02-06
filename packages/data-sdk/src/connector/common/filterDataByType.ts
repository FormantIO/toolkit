import { IStreamData, StreamType } from "../../main";

export function filterDataByType<T extends StreamType>(
  datas: IStreamData[],
  type: T
): IStreamData<T>[] {
  return datas.filter((_) => _.type === type) as IStreamData<T>[];
}
