export function toJsonLines(objects: any[]): string {
  return objects.reduce((acc, _) => `${acc}${JSON.stringify(_)}\n`, "");
}
