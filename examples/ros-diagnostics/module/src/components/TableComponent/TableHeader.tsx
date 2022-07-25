import { Component, FC, ReactNode } from "react";
import styles from "./TableHeader.module.scss";
import { Icon } from "@formant/ui-sdk";

interface ITableHeader {
  headers: string[];
  showConfig?: () => void;
}

export const TableHeader: FC<ITableHeader> = ({ headers, showConfig }) => {
  return (
    <thead>
      <tr>
        {headers.map((_, idx) => {
          return (
            <th key={idx} className={styles.header}>
              {_}
            </th>
          );
        })}
        <th
          style={{
            cursor: "pointer",
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
          }}
          onClick={showConfig}
          className={styles.header}
        >
          {<Icon name="settings" />}
        </th>
      </tr>
    </thead>
  );
};
