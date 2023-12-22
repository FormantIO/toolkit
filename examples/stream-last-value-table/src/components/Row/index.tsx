import { FC, useEffect, useState } from "react";
import styles from "./index.module.scss";
import { IConfiguration, Status } from "types";
import { useFormant } from "@formant/ui-sdk";
interface IRowProps {
  leftValue: string;
  rightValue: string;
  state: Status;
  height?: number | null;
  fullWidth?: boolean;
  idx?: number;
}

const WINDOW_TELEOP_MIN_WIDTH = 301;
const TIMEOUT_DEBOUNCER = 100;

export const Row: FC<IRowProps> = (props: any) => {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const { leftValue, rightValue, state, height, fullWidth } = props;
  const [teleopMode, setTeleopMode] = useState(false);

  useEffect(() => {
    const handleResize = () => {
      let timeoutId: any = undefined;
      clearTimeout(timeoutId);
      timeoutId = setTimeout(() => {
        if (window.innerWidth < WINDOW_TELEOP_MIN_WIDTH) {
          setTeleopMode(true);
        } else {
          setTeleopMode(false);
        }
      }, TIMEOUT_DEBOUNCER);

      return () => clearTimeout(timeoutId);
    };

    if (window.innerWidth < WINDOW_TELEOP_MIN_WIDTH) {
      setTeleopMode(true);
    } else {
      setTeleopMode(false);
    }

    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
    };
  }, []);

  return (
    <div
      style={{
        width: !!fullWidth ? "100vw" : teleopMode ? "50vw" : "100%",
        height: !!height ? (height < 30 ? 30 : height) : 40,
        borderRight: "solid black 1px",
      }}
      className={styles.row}
    >
      <div
        style={{
          width: !!fullWidth && teleopMode ? "40%" : "50%",
          overflow: "hidden",
          fontSize: !!config?.fontSize ? `${config?.fontSize}px` : "14px",
          backgroundColor: rightValue === "Header" && "#1C1E2D",
        }}
        className={styles["row-stream"]}
      >
        <span>{leftValue}</span>
      </div>
      {rightValue === "Header" && (
        <div
          style={{
            width: !!fullWidth && teleopMode ? "60%" : "50%",
            backgroundColor: rightValue === "Header" && "#1C1E2D",
            height: "100%",
          }}
        >
          {" "}
        </div>
      )}
      {rightValue !== "Header" && (
        <div
        style={{
          width: !!fullWidth && teleopMode ? "60%" : "50%",
        }}
          className={`${styles["row-value"]} ${
            styles[
              state === "warning"
                ? "warning"
                : state === "good"
                ? "good"
                : "offline"
            ]
          }`}
        >
          <span
            style={{
              fontSize: !!config?.fontSize ? `${config?.fontSize}px` : "14px",
            }}
          >
            {rightValue}
          </span>
        </div>
      )}
    </div>
  );
};
