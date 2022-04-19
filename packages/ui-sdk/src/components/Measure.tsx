import { duration } from "../../../common/duration";
import * as React from "react";
import { HTMLAttributes, useEffect, useRef } from "react";

interface IMeasureProps {
  onResize: (width: number, height: number) => void;
  dimension?: IMeasureDimension;
  children?: React.ReactNode;
}

export type IMeasureDimension = "client" | "offset" | "scroll";

const measures = new Set<() => void>();

let measureInterval: number | undefined;

export function Measure(props: IMeasureProps & HTMLAttributes<HTMLDivElement>) {
  const { dimension, onResize, children } = props;
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    let lastWidth = 0;
    let lastHeight = 0;
    const performMeasure = () => {
      if (!ref.current) {
        return;
      }

      const width =
        (dimension === "offset" && ref.current.offsetWidth) ||
        (dimension === "scroll" && ref.current.scrollWidth) ||
        ref.current.clientWidth;

      const height =
        (dimension === "offset" && ref.current.offsetHeight) ||
        (dimension === "scroll" && ref.current.scrollHeight) ||
        ref.current.clientHeight;

      if (lastWidth !== width || lastHeight !== height) {
        lastWidth = width;
        lastHeight = height;
        onResize(width, height);
      }
    };
    measures.add(performMeasure);
    performMeasure();
    if (!measureInterval) {
      setInterval(() => measures.forEach((_) => _()), 0.5 * duration.second);
    }
    return () => {
      measures.delete(performMeasure);
      if (measures.size === 0) {
        clearInterval(measureInterval);
      }
    };
  }, []);

  return (
    <div ref={ref} {...props}>
      {children}
    </div>
  );
}
