import { Component, ReactNode } from "react";
import styles from "./TableSection.module.scss";

interface ITableSectionProps {
  title: string;
  rowSpan: number;
}

export class TableSection extends Component<ITableSectionProps> {
  render(): ReactNode {
    return (
      <td className={styles.section} rowSpan={this.props.rowSpan}>
        {this.props.title}
      </td>
    );
  }
}
