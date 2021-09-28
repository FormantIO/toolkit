export function defined<T>(value: T | undefined, errorMessage?: string): T {
  if (value !== undefined) {
    return value;
  }
  throw new Error(errorMessage || "Value is undefined");
}

export function notNull<T>(value: T | null, errorMessage?: string): T {
  if (value !== null) {
    return value;
  }
  throw new Error(errorMessage || "Value is null");
}

export function definedAndNotNull<T>(
  value: T | undefined | null,
  errorMessage?: string
): T {
  return notNull(defined(value, errorMessage), errorMessage);
}
