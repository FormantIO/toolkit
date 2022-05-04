import { Icon, Typography } from "@alenjdev/ui-sdk";
import { FC } from "react";
import styles from "./App.module.scss";
interface IDeviceProps {
  latitude?: string | number;
  longitude?: string | number;
  weight?: string | number;
}

export const DeviceCard: FC<IDeviceProps> = ({
  latitude,
  longitude,
  weight,
}) => {
  return (
    <div
      id="card"
      className={`${styles.card} ${
        weight! < 10
          ? styles["card-low"]
          : weight! < 30
          ? styles["card-medium"]
          : weight! < 60
          ? styles["card-high"]
          : styles.card
      }`}
    >
      <Typography type="h2">
        {`${latitude}, ${longitude}` || `High -86.788869, 36.148066`}
      </Typography>
      <div className={styles["card-name"]}>
        <Icon color="#BAC4E2" name="device" size="small" />
        <Typography type="h3">Astro</Typography>
      </div>
      <Typography type="h4">{`Points of interest: ${
        weight || "-"
      }`}</Typography>
    </div>
  );
};
