import { useEffect, useState } from "react";
import reactLogo from "./assets/react.svg";
import "./App.css";
import { useDevice, DoughnutChart, BarChart } from "@formant/ui-sdk";
import { App as FormantApp } from "@formant/data-sdk";
import { Authentication, Fleet, Device, IEvent } from "@formant/data-sdk";

const week_count = 5;
let annotation_data: number[] = [];
let date_labels: string[] = [];
let max_weekly_avg_errors = 0;
const average = (array) => array.reduce((a, b) => a + b) / array.length;

export function WeeklyAvgAlertPerRun() {
  const [annotations, setAnnotations] = useState<any[] | undefined>();
  useEffect(() => {
    getAnnotationData();
  }, []);

  const getAnnotationData = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      annotation_data = [];
      date_labels = [];
      // for each week
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
        let dates = first_day_readable + "-" + last_day_readable;

        console.log(first_day_readable);

        // Get all annotations for the current week

        const eventQueryResponse = await Fleet.queryEvents({
          deviceIds: [currentDevice.id],
          eventTypes: ["annotation"],
          start: first_day_iso,
          end: last_day_iso,
        });
        let annotations = eventQueryResponse;

        // If the week had annotations, run through them each and get the max alert count per annotation

        if (annotations.length > 0) {
          let weekly_alert_count = [];

          // for each annotation in the current week
          for (let j = 0; j < annotations.length; j++) {
            let job_start_time = annotations[j].time;
            let job_end_time = annotations[j].endTime;

            // query the Active_Alerts stream for the duration of the annotation
            const annotation_alerts = await Fleet.queryTelemetry({
              deviceIds: [currentDevice.id],
              start: job_start_time,
              end: job_end_time,
              names: ["Active_Alerts"],
            });
            console.log("here are the active alerts for this annotation");
            // console.log(annotation_alerts);
            let max_alerts_per_annotation = 0;
            if (annotation_alerts.length > 0) {
              // since it's only one stream for one device, we get index 0 and its all the stream data
              let alert_data = annotation_alerts[0].points;
              // console.log(alert_data);
              for (let k = 0; k < alert_data.length; k++) {
                // for each Active_Alert datapoint, see if the length of the array of data (index 0 is timestamp, 1 is the data and since it's a numeric set this is an array of dicts)
                if (alert_data[k][1].length > max_alerts_per_annotation) {
                  max_alerts_per_annotation = alert_data[k][1].length;
                }
              }
              // add the max_alert_per_annotation to the array, should be same length as the length of annotations
            }
            weekly_alert_count.unshift(max_alerts_per_annotation);
            console.log(max_alerts_per_annotation);
            console.log("=====");
          }
          const avg_weekly_alert_count = average(weekly_alert_count);
          if (avg_weekly_alert_count > max_weekly_avg_errors) {
            max_weekly_avg_errors = avg_weekly_alert_count;
          }
          annotation_data.unshift(avg_weekly_alert_count);

          date_labels.unshift(dates);
        }
        console.log(last_day_readable);
      }
      setAnnotations(annotation_data);
    }
  };

  return (
    <>
      <BarChart
        labels={date_labels}
        data={annotation_data}
        xMax={max_weekly_avg_errors * 1.25}
        height={600}
        width={600}
      />
    </>
  );
}
