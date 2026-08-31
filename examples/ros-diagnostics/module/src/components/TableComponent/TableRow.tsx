import { Component, ReactNode } from "react";
import styles from "./TableRow.module.scss";
interface ITableRowProps {
  children: ReactNode;
}

export class TableRow extends Component<ITableRowProps> {
  render(): ReactNode {
    return <tr className={styles.row}>{this.props.children}</tr>;
  }
}
