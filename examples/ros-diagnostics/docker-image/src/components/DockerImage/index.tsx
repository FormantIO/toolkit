import { Component, ReactNode } from "react";
import styles from "./index.module.scss";

interface IDockerProps {
  latestImage: string;
  getLatestImage: () => void;
}

export class Docker extends Component<IDockerProps> {
  render(): ReactNode {
    return (
      <div className={styles.container}>
        <div className={styles["container-comand"]}>
          <p className={styles["container-date"]}>2:09:57 pm</p>
          <p className={styles["container-text"]}>{this.props.latestImage}</p>
        </div>
        <button
          className={styles["container-button"]}
          onClick={this.props.getLatestImage}
        >
          Update
        </button>
      </div>
    );
  }
}
