import { MessageRow } from "../MessageRow";
import { IStatus } from "../../types/types";
import { FC } from "react";
import styled from "@emotion/styled";
import { mediaQueries } from "../../styles/breakpoints";

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
    <Container
      style={{
        minWidth: active === null ? "100%" : 500,
      }}
    >
      {messages
        .sort((a, b) => a.name.localeCompare(b.name))
        .map((_) => (
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
    </Container>
  );
};

const Container = styled.div`
  display: flex;
  flex-direction: column;
  background-color: #282f45;
  min-width: 500px;
  overflow: auto;
  ${mediaQueries.small} {
    width: 100vw;
  }
`;
