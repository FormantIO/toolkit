import React, { useState } from "react";
import Calendar from "react-calendar";
import styles from "./index.module.scss";
export const FormantCalendar = () => {
  const [value, onChange] = useState(new Date());
  const formantMonthYear = (_, date) => (date, "MMMM");
  const formatShortWeekday = (_, date) =>
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
