import {
  IVector2,
  IShutdownPromiseInterval,
  IPromiseIntervalOptions,
} from "./types";

type Timeout = ReturnType<typeof setTimeout>;

export const debounce = (callback: (...args: any[]) => void, time: number) => {
  let timeout: Timeout;

  return (...args: any[]) => {
    if (timeout) {
      clearTimeout(timeout);
    }
    timeout = setTimeout(() => callback(...args), time);
  };
};

export const range = (start: number, end: number) =>
  end <= start ? [] : new Array(end - start).fill(0).map((_, i) => i + start);

export const deadzone = (value: number, threshold = 0.1) => {
  const deadzoneValue =
    Math.abs(value) < threshold
      ? 0
      : value < 0
      ? value + threshold
      : value - threshold;
  return deadzoneValue * (1 / (1 - threshold));
};

export function vectorLength(v: IVector2) {
  const { x, y } = v;
  return Math.sqrt(x * x + y * y);
}

export function clampVectorLength(v: IVector2, l: number) {
  const length = vectorLength(v);
  const { x, y } = v;
  if (length > l) {
    return { x: (x / length) * l, y: (y / length) * l };
  } else {
    return v;
  }
}

export function clamp(value: number, min = -1, max = 1) {
  return value < min ? min : value > max ? max : value;
}

export const defaultSameValueJoystickDebounce = 0.5;
export const defaultNewValueJoystickDebounce = 0.1;
export const lowSameValueJoystickDebounce = 0.1;
export const lowNewValueJoystickDebounce = 0.05;

const millisecond = 1;
const second = 1000;
const minute = 60 * second;
const hour = 60 * minute;
const day = 24 * hour;
const week = 7 * day;
const month = 30 * day;
const year = 365 * day;

export const duration = {
  millisecond,
  second,
  minute,
  hour,
  day,
  week,
  month,
  year,
} as const;

export function setPromiseInterval({
  func,
  delay,
  immediate = false,
}: IPromiseIntervalOptions): IShutdownPromiseInterval {
  let stopped = false;
  let timeout = setTimeout(g, immediate ? 0 : delay);
  let promise = Promise.resolve();

  async function g() {
    if (stopped) {
      return;
    }
    const t0 = new Date().getTime();
    try {
      promise = func();
      await promise;
    } finally {
      if (!stopped) {
        const t1 = new Date().getTime();
        timeout = setTimeout(g, Math.max(delay - (t1 - t0), 0));
      }
    }
  }

  return {
    async stop() {
      stopped = true;
      clearTimeout(timeout);
      await promise;
    },
  };
}

export function addVectors(a: IVector2, b: IVector2) {
  return { x: a.x + b.x, y: a.y + b.y };
}
