import React from "react";
import loading from "../../images/loading.png";
import styles from "./index.module.scss";

export const LoadingIndicator = () => {
  return (
    <div className={styles["formant-loading-indicator"]}>
      <img src={loading} />
    </div>
  );
};
