import { Timeout } from "./Timeout";

export const throttle = <T>(callback: (...args: T[]) => void, time: number) => {
  let timeout: Timeout | undefined;
  let lastTime: number | undefined;

  return (...args: T[]) => {
    const interval =
      lastTime !== undefined ? Math.max(0, lastTime + time - Date.now()) : 0;

    if (args.length === 0 && timeout) {
      // Performance fix: If args don't matter we don't have to reset the timeout
      return;
    }

    if (timeout) {
      clearTimeout(timeout);
    }

    timeout = setTimeout(() => {
      callback(...args);
      timeout = undefined;
      lastTime = Date.now();
    }, interval);
  };
};
