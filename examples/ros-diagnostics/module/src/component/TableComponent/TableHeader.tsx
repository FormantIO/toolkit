import { Component, ReactNode } from "react";
import styles from "./TableHeader.module.scss";

interface ITableHeader {
  headers: string[];
}

export class TableHeader extends Component<ITableHeader> {
  render(): ReactNode {
    return (
      <thead>
        {this.props.headers.map((_, idx) => {
          return (
            <th key={idx} className={styles.header}>
              {_}
            </th>
          );
        })}
      </thead>
    );
  }
}
