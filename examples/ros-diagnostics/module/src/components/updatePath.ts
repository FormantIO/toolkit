import { setWith, clone } from "lodash";

export const updatePath = <T extends object>(
  state: T,
  path: string | string[],
  value: any
): T => setWith<T>(clone(state), path, value, clone);
