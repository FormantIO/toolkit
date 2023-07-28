import { MessageRow } from "../MessageRow";
import { DiagnosticStatusMessage, IStatus } from "../../types/types";
import { FC } from "react";
import styles from "./index.module.scss";
import styled from "@emotion/styled";

interface IMessageTable {
  messages: IStatus[];
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
        // <Row>
        //   {_.name}
        // </Row>
        <MessageRow
          key={_.name}
          level={_.level}
          active={active}
          setActive={() => {
            handleSetActive(_.name);
          }}
          message={_.name}
        />
      ))}
    </div>
  );
};

const Row = styled.div`
  height: 40px;
`;
