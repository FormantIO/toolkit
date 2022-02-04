import { Component, ReactNode } from "react";
import styles from "./Topic.module.scss";
import { CheckedIcon } from "../Icons/CheckedIcon";
import { ErrorIcon } from "../Icons/ErrorIcon";
import loading from "../../../src/components/images/loading.png";

interface ITopicProps {
  name: string;
  isLoading?: boolean;
  topicState: "good_standing" | "not_found";
}

interface ITopicState {
  isLoading: boolean;
}

export class Topic extends Component<ITopicProps, ITopicState> {
  render(): ReactNode {
    return (
      <div className={styles.topic}>
        <span className={styles["topic-name"]}>{this.props.name}</span>
        <div className="topic-connection">
          {this.props.isLoading ? (
            <img className={styles["topic-image"]} src={loading} />
          ) : this.props.topicState === "good_standing" ? (
            <CheckedIcon />
          ) : (
            <ErrorIcon />
          )}
        </div>
      </div>
    );
  }
}
