import loading from "../../images/loading.png";
import styles from "./index.module.scss";

export const LoadingIndicator = () => {
  window.addEventListener("resize", () => {
    const loadingSpinner = document.getElementById("loading-spinner");
  });
  return (
    <div id="loading-spinner" className={styles.loading}>
      <img src={loading} />
    </div>
  );
};
