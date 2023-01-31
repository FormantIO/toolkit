import { Icon, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import styles from "./index.module.scss";
import { SeverityLevel } from "../../types/types";
interface IMesageRowProps {
  message: string;
  active: string | null;
  setActive: () => void;
  level: number | string;
}

export const MessageRow: FC<IMesageRowProps> = ({
  message,
  setActive,
  active,
  level,
}) => {
  return (
    <div className={styles["message-row"]} onClick={setActive}>
      <div
        className={`${styles["inner-message"]} ${
          active === message ? styles["inner-message-active"] : ""
        }`}
      >
        <div className={styles.text}>
          <Icon
            name={
              level === 0 || level === "b'\\x00'"
                ? "check"
                : level === 1 || level === "b'\\x01'"
                ? "warning"
                : level === 2 || level === "b'\\x02'"
                ? "critical"
                : "online"
            }
          />
          <Typography
            sx={{
              color: "white",
              width: "auto",
            }}
          >
            {message}
          </Typography>
        </div>
        <div className={styles.chevron}>
          <Icon name="chevron-right" />
        </div>
      </div>
    </div>
  );
};
