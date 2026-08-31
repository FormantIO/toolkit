import { Component, ReactNode } from "react";
import styles from "./TableDataCell.module.scss";

interface ITableDataCellProps {
  type?: "unknown" | "good" | "bad";
  content: string | number | ReactNode;
  center?: boolean;
}

export class TableDataCell extends Component<ITableDataCellProps> {
  render(): ReactNode {
    return (
      <td
        className={`${styles["data-cell"]} ${
          styles[`data-cell-${this.props.type}`]
        } ${this.props.center && styles["data-cell-center"]} `}
      >
        {this.props.content}
      </td>
    );
  }
}
