/**
 * Used to fire-and-forget a promise.
 * This is useful as a function wrapper to suppress floating promise warnings.
 */
export function fork<T>(_: Promise<T>): void {
  return undefined;
}
