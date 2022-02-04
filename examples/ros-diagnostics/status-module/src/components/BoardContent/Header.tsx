import { Component, ReactNode } from "react";
import styles from "./Header.module.scss";

interface IHeaderProps {
  title: string;
}

export class Header extends Component<IHeaderProps> {
  render(): ReactNode {
    return (
      <h2 className={styles.header}>
        {this.props.title.charAt(0).toUpperCase() + this.props.title.slice(1)}
      </h2>
    );
  }
}
