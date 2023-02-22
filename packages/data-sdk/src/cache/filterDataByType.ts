import { IStreamData } from "../model/IStreamData";
import { StreamType } from "../model/StreamType";

export function filterDataByType(
  datas: IStreamData[],
  type: StreamType[]
): IStreamData[] {
  return datas.filter((_) => type.includes(_.type)) as IStreamData[];
}
