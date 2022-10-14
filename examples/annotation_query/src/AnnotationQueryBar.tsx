import { useEffect, useState } from "react";
import reactLogo from "./assets/react.svg";
import "./App.css";
import { useDevice, DoughnutChart, BarChart } from "@formant/ui-sdk";
import { App as FormantApp } from "@formant/data-sdk";
import { Authentication, Fleet, Device, IEvent } from "@formant/data-sdk";

let t1 = new Date("2022-10-05T00:00:00");

let t2 = new Date("2022-10-06T14:30:00");

interface AnnotationQueryProps {
  start_date: Date;
  end_date: Date;
  operation_status: string[];
  // tags?: string[];
  // annotationTemplateIds?: string[];
  // severities?: [];
}

export function AnnotationQueryBar(props: AnnotationQueryProps) {
  const { start_date, end_date, operation_status } = props;
  const start_date_iso = start_date.toISOString();
  const end_date_iso = end_date.toISOString();
  const [annotations, setAnnotations] = useState<IEvent[] | undefined>();
  // when to retain state; if they are asynchronous; do they change asynchronously (if this is a background process or a user interaction) so this includes button clicks or entering text
  // const [results, setResults] = useState<number[]>()
  useEffect(() => {
    getAnnotationData();
  }, []);
  //getAnnotationData runs when any variables change in the [] in line above

  const getAnnotationData = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      const eventQueryResponse = await Fleet.queryEvents({
        deviceIds: [currentDevice.id],
        eventTypes: ["annotation"],
        start: start_date_iso,
        end: end_date_iso,
        tags: { operation_status: operation_status },
      });
      setAnnotations(eventQueryResponse);
    }
  };

  const numAnnotations = annotations?.length;
  const numSuccessAnnotations = annotations?.filter(
    (_) => _.type === "annotation" && _.tags?.operation_status === "Success"
  ).length;
  const numFailedAnnotations = annotations?.filter(
    (_) => _.type === "annotation" && _.tags?.operation_status === "Aborted"
  ).length;
  const result_array = [
    numAnnotations,
    numSuccessAnnotations,
    numFailedAnnotations,
  ];

  return (
    <>
      <BarChart
        labels={["Successes", "Failures"]}
        data={[numSuccessAnnotations, numFailedAnnotations]}
        xMax={10}
        height={400}
        width={400}
      />
    </>
  );
}

// export default App
