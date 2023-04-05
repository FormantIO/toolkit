import { IsoDate } from "./IsoDate";

export interface ISqlRow {
  [key: string]: string | IsoDate | number | boolean | null;
  tableName: string;
}
