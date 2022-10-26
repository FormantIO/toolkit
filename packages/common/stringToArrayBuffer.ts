export function stringToArrayBuffer(input: string): Uint8Array {
  return Uint8Array.from(atob(input), (_) => _.charCodeAt(0));
}
