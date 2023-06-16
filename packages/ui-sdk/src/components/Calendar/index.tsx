import React, { useState } from "react";
// @ts-ignore
import Calendar from "react-calendar";
import { format } from "date-fns";

import styles from "./index.module.scss";
export const FormantCalendar = () => {
  const [value, onChange] = useState(new Date());
  const formantMonthYear = (_, date: Date) => format(date, "MMMM");
  const formatShortWeekday = (_, date: Date) =>
    date.toLocaleDateString("en-US", { weekday: "narrow" });

  return (
    <div>
      <Calendar
        className={styles.calendar}
        onChange={onChange}
        value={value}
        formatMonthYear={formantMonthYear}
        formatShortWeekday={formatShortWeekday}
        showNeighboringMonth={false}
      />
    </div>
  );
};
