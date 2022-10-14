import { useEffect, useState } from "react";
import reactLogo from "./assets/react.svg";
import "./App.css";
import { useDevice, DoughnutChart, BarChart } from "@formant/ui-sdk";
import { App as FormantApp } from "@formant/data-sdk";
import { Authentication, Fleet, Device, IEvent } from "@formant/data-sdk";

const week_count = 5;
let annotation_data: number[] = [];
let date_labels: string[] = [];

export function AnnotationAbortBarWeekly() {
  const [annotations, setAnnotations] = useState<any[] | undefined>();
  useEffect(() => {
    getAnnotationData();
  }, []);

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
          tags: { operation_status: ["Aborted"] },
        });
        let dates = first_day_readable + "-" + last_day_readable;
        let annotation_filtered = eventQueryResponse.length;
        annotation_data.unshift(annotation_filtered);
        date_labels.unshift(dates);
      }
      setAnnotations(annotation_data);
    }
  };

  return (
    <>
      <BarChart
        labels={date_labels}
        data={annotation_data}
        xMax={10}
        height={400}
        width={400}
      />
    </>
  );
}
