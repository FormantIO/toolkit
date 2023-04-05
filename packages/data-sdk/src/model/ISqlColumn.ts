export interface ISqlColumn {
  name: string;
  tableName: string;
  isNullable?: boolean;
  dataType: "string" | "timestamp" | "boolean" | "number" | "null";
}
