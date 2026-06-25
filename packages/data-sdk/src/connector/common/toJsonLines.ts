export function toJsonLines(objects: unknown[]): string {
  return objects.reduce((acc, obj) => `${acc}${JSON.stringify(obj)}\n`, "");
}
