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
