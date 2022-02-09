import { Component, ReactNode } from "react";
import styles from "./index.module.scss";

interface IDockerProps {
  latestImage: string;
  getLatestImage: () => void;
  image: string;
  date: string;
}

export class Docker extends Component<IDockerProps> {
  render(): ReactNode {
    return (
      <div className={styles.container}>
        <div className={styles["container-comand"]}>
          <p className={styles["container-date"]}>{this.props.date}</p>
          <p className={styles["container-text"]}>{this.props.image}</p>
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
