import { Component, ReactNode } from "react";
import styles from "./TopicBoard.module.scss";

interface ITopicBoardProps {
  children: ReactNode;
}

export class TopicBoard extends Component<ITopicBoardProps> {
  render(): ReactNode {
    return <div className={styles.board}>{this.props.children}</div>;
  }
}
