import { useEffect, useState } from "react";
import reactLogo from "./assets/react.svg";
import "./App.css";
import { useDevice, DoughnutChart, BarChart } from "@formant/ui-sdk";
import { App as FormantApp } from "@formant/data-sdk";
import { Authentication, Fleet, Device, IEvent } from "@formant/data-sdk";

//maybe we pass in the date range to this app, so that we can re-use this for daily/weekly/whatever
let t1 = new Date("2022-10-05T00:00:00");
// const start_date_iso = start_date.toISOString()
// const start_date_readable = start_date.toLocaleString()

let t2 = new Date("2022-10-06T14:30:00");
// const end_date_iso = end_date.toISOString()
// const end_date_readable = end_date.toLocaleString()

// need some help with getting parameters into the app... start_date: Date = t1, end_date: Date = t2

function App() {
  const start_date = t1;
  const end_date = t2;
  const start_date_iso = start_date.toISOString();
  const start_date_readable = start_date.toLocaleString();
  const end_date_iso = end_date.toISOString();
  const end_date_readable = end_date.toLocaleString();

  const [events, setEvents] = useState<IEvent[] | undefined>();
  useEffect(() => {
    getMissionData();
  }, []);
  //getMissionData runs when any variables change in the [] in line above
  const getMissionData = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      const eventQueryResponse = await Fleet.queryEvents({
        deviceIds: [currentDevice.id],
        tags: { operation_status: ["Success", "Aborted", "None"] },
        eventTypes: ["annotation"],
        start: start_date_iso,
        end: end_date_iso,
      });
      setEvents(eventQueryResponse);
    }
  };

  // var arr = events?.tags?.operation_status
  const numEvents = events?.length;
  const numSuccessEvents = events?.filter(
    (_) => _.type === "annotation" && _.tags?.operation_status === "Success"
  ).length;
  const numFailedEvents = events?.filter(
    (_) => _.type === "annotation" && _.tags?.operation_status === "Aborted"
  ).length;
  //filter returns an array where the following predicate is true; _ is any element; check that type is annotation and operation.status is success

  //enable date parameters to be passed into this app so i can reuse this app

  return (
    <div className="App">
      {start_date_readable} -- {end_date_readable}
      <DoughnutChart
        height={400}
        width={400}
        labels={["Successes", "Failures"]}
        data={[numSuccessEvents, numFailedEvents]}
      />
      <BarChart
        labels={["Successes", "Failures"]}
        data={[numSuccessEvents, numFailedEvents]}
        xMax={10}
        height={400}
        width={400}
      />
    </div>
  );
}

export default App;
