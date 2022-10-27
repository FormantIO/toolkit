import { useEffect, useState } from "react";
import reactLogo from "./assets/react.svg";
import "./App.css";
import { useDevice, DoughnutChart, BarChart } from "@formant/ui-sdk";
import { App as FormantApp } from "@formant/data-sdk";
import { Authentication, Fleet, Device, IEvent } from "@formant/data-sdk";
import { AnnotationQueryDoughnut } from "./AnnotationQueryDoughnut";
import { AnnotationQueryBar } from "./AnnotationQueryBar";
import { AnnotationSuccessBarWeekly } from "./WeeklyJobSuccessBar";
import { AnnotationAbortBarWeekly } from "./WeeklyJobAbortBar";
import { AnnotationDurationBarWeekly } from "./WeeklyJobDurationBar";
import { WeeklyAvgAlertPerRun } from "./WeeklyAvgAlertPerRun";

let t1 = new Date("2022-10-04T00:00:00");
const t1_readable = t1.toLocaleString();

let t2 = new Date("2022-10-10T14:30:00");
const t2_readable = t2.toLocaleString();

function App() {
  let today = new Date();
  let curr = new Date("2022-10-04T00:00:00");
  let first = curr.getDate() - curr.getDay();
  let last = first + 6;
  let firstday = new Date(curr.setDate(first)).toUTCString();
  let lastday = new Date(curr.setDate(last)).toUTCString();
  const Annotation_params = {
    start_date: t1,
    end_date: t2,
    operation_status: ["Success", "Aborted"],
  };

  return (
    <div className="App">
      {AnnotationQueryDoughnut()}
      {AnnotationQueryBar()}
      Completed Jobs per Week
      {AnnotationSuccessBarWeekly()}
      Aborted Jobs per Week
      {AnnotationAbortBarWeekly()}
      Weekly Average Runtime (Minutes)
      {AnnotationDurationBarWeekly()}
      Weekly Average Alerts Per Run
      {WeeklyAvgAlertPerRun()}
    </div>
  );
}

export default App;
