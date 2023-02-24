import { Component, ReactNode } from "react";
import styles from "./Table.module.scss";

interface ITableProps {
  children: ReactNode;
  columns: number;
}

export class Table extends Component<ITableProps> {
  render(): ReactNode {
    return (
      <table
        style={{
          gridTemplateColumns: `repeat(${this.props.columns}, 1fr)`,
        }}
        className={styles.table}
      >
        {this.props.children}
      </table>
    );
  }
}
