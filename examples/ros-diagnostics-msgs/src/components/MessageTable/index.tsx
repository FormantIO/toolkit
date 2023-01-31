import { MessageRow } from "../MessageRow";
import { DiagnosticStatusMessage } from "../../types/types";
import { FC } from "react";
import styles from "./index.module.scss";

interface IMessageTable {
  messages: DiagnosticStatusMessage[];
  active: string | null;
  handleSetActive: (_: string) => void;
}

export const MessageTable: FC<IMessageTable> = ({
  messages,
  active,
  handleSetActive,
}) => {
  return (
    <div
      className={styles["message-table"]}
      style={{
        minWidth: active === null ? "100%" : 500,
      }}
    >
      {messages.map((_) => (
        <MessageRow
          key={_.status[0].name}
          level={_.status[0].level}
          active={active}
          setActive={() => {
            handleSetActive(_.status[0].name);
          }}
          message={_.status[0].name}
        />
      ))}
    </div>
  );
};
