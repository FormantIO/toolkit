export type RequireKeys<T> = { [K in keyof T & {}]: T[K] };
