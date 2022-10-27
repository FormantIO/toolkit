import { useEffect, useState } from "react";
import reactLogo from "./assets/react.svg";
import "./App.css";
import { useDevice, DoughnutChart, BarChart } from "@formant/ui-sdk";
import { App as FormantApp } from "@formant/data-sdk";
import { Authentication, Fleet, Device, IEvent } from "@formant/data-sdk";

const week_count = 5;
let annotation_data: number[] = [];
let date_labels: string[] = [];
let max_duration = 0;
const average = (array) => array.reduce((a, b) => a + b) / array.length;

export function AnnotationDurationBarWeekly() {
  const [annotations, setAnnotations] = useState<any[] | undefined>();
  useEffect(() => {
    getAnnotationData();
  }, []);

  function durationToMinutes(string: String) {
    const times = string.split(":");
    const hrs = Number(times[0]) * 60;
    const mins = Number(times[1]);
    const secs = Number(times[2]) / 60;
    const duration_mins = hrs + mins + secs;
    return duration_mins;
  }

  const getAnnotationData = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      annotation_data = [];
      date_labels = [];
      for (let i = 0; i < week_count; i++) {
        let today = new Date();
        let date_shift = 7 * i;
        today.setDate(today.getDate() - date_shift);
        let t1 = today.getDate() - today.getDay();
        let t2 = t1 + 6;

        let first_day = new Date(today.setDate(t1));
        let first_day_iso = first_day.toISOString();
        let first_day_readable = first_day.toLocaleDateString();

        let last_day = new Date(today.setDate(t2));
        let last_day_iso = last_day.toISOString();
        let last_day_readable = last_day.toLocaleDateString();

        const eventQueryResponse = await Fleet.queryEvents({
          deviceIds: [currentDevice.id],
          eventTypes: ["annotation"],
          start: first_day_iso,
          end: last_day_iso,
        });
        let dates = first_day_readable + "-" + last_day_readable;
        let job_durations = eventQueryResponse;
        let average_weekly_duration = 0;
        if (job_durations.length > 0) {
          let weekly_durations = [];
          for (let j = 0; j < job_durations.length; j++) {
            let string_duration = job_durations[j].tags["operation_duration"];
            let num_duration = durationToMinutes(string_duration);
            weekly_durations.push(num_duration);
          }

          average_weekly_duration = average(weekly_durations);
          if (average_weekly_duration > max_duration) {
            max_duration = average_weekly_duration;
          }

          annotation_data.unshift(average_weekly_duration);
          date_labels.unshift(dates);
        }
      }
      setAnnotations(annotation_data);
    }
  };

  return (
    <>
      <BarChart
        labels={date_labels}
        data={annotation_data}
        xMax={max_duration * 1.1}
        height={600}
        width={600}
      />
    </>
  );
}
