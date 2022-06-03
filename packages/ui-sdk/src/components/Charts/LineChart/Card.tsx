import styles from "./card.module.scss";
import React from "react";
import { Icon } from "../../../Icon";
export const Card: React.FC = () => {
  return (
    <div id="chartjs-tooltip" className={styles.card}>
      <div className={styles["text-container"]}>
        <span className={styles.label}>
          X: <span className={styles["card-x"]} id="tooltop-text"></span>,
        </span>
        <span className={styles.label}>
          Y: <span className={styles["card-y"]} id="tooltop-text-2"></span>
        </span>
        <span className={styles.date}>
          {`${new Date().toISOString().slice(11, -5)} pm`}
        </span>
      </div>
      <div style={{ marginTop: 5 }} className={styles["text-container"]}>
        <Icon name="device" />
        <span className={styles["card-astro"]}>Astro</span>
      </div>
    </div>
  );
};
