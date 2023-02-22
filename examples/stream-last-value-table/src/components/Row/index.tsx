import { FC, useEffect, useState } from "react";
import styles from "./index.module.scss";
import { Status } from "types";
interface IRowProps {
  leftValue: string;
  rightValue: string;
  state: Status;
  height?: number | null;
  fullWidth?: boolean;
}

export const Row: FC<IRowProps> = (props: any) => {
  const { leftValue, rightValue, state, height, fullWidth } = props;
  const [teleopMode, setTeleopMode] = useState(false);

  useEffect(() => {
    if (window.innerWidth < 301) setTeleopMode(true);
  }, []);

  window.addEventListener("resize", () => {
    if (window.innerWidth < 301) {
      setTeleopMode(true);
      return;
    }
    setTeleopMode(false);
  });

  return (
    <div
      style={{
        width: !!fullWidth ? "100vw" : teleopMode ? "50vw" : "100%",
        height: !!height ? (height < 30 ? 30 : height) : 40,
      }}
      className={styles.row}
    >
      <div
        style={{
          width: !!fullWidth && teleopMode ? "40%" : "50%",
          overflow: "hidden",
        }}
        className={styles["row-stream"]}
      >
        <span>{leftValue}</span>
      </div>
      <div
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
        <span style={{}}>{rightValue}</span>
      </div>
    </div>
  );
};
