export function defined<T>(value: T | undefined): T {
  if (value !== undefined) {
    return value;
  }
  throw new Error("Value is undefined");
}

export function notNull<T>(value: T | null): T {
  if (value !== null) {
    return value;
  }
  throw new Error("Value is null");
}

export function definedAndNotNull<T>(value: T | undefined | null): T {
  return notNull(defined(value));
}
