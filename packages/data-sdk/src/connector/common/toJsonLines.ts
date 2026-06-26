export function toJsonLines(objects: unknown[]): string {
  return objects.reduce<string>(
    (acc, obj) => `${acc}${JSON.stringify(obj)}\n`,
    ""
  );
}
