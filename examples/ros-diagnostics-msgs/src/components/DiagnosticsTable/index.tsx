import { Typography, Tooltip } from "@formant/ui-sdk";
import { FC } from "react";
import { IconButton } from "../IconButton";
import "../../index.css";

import styled from "@emotion/styled";
import { mediaQueries } from "../../styles/breakpoints";
interface IDiagnosticsTableProps {
  diagnosticsDetails: any[] | null;
  active: string | null;
}

export const DiagnosticsTable: FC<IDiagnosticsTableProps> = ({
  diagnosticsDetails,
  active,
}) => {
  const handleBack = () => {
    const table = document.getElementById("diagnostics-table")!;
    table.classList.remove("slide-out-left");
    table.classList.add("slide-out-right");
    table.style.visibility = "hidden";
  };

  return (
    <Container id="diagnostics-table">
      <Header>
        <IconButton onClick={handleBack} />
        <Typography sx={{ color: "white" }}>{active}</Typography>
      </Header>
      {diagnosticsDetails
        ?.sort((a, b) => a.key.localeCompare(b.key))
        .map((_) => {
          return (
            <Row key={_.key}>
              <Tooltip title={_.key}>
                <Key>{_.key}</Key>
              </Tooltip>
              <Tooltip title={_.value}>
                <Value>{_.value}</Value>
              </Tooltip>
            </Row>
          );
        })}
    </Container>
  );
};

const Key = styled.p`
  margin: 0;
  font-style: normal;
  font-weight: normal;
  font-size: 1rem;
  line-height: 1.688rem;
  letter-spacing: 0.063rem;
  font-feature-settings: "zero" on;
  color: white;
  padding: 8px;
  white-space: nowrap;
  min-width: 300px;
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 290px;
  white-space: nowrap;
  max-width: 141px;
  overflow: hidden;
  text-overflow: ellipsis;
`;

const Value = styled.p`
  margin: 0;
  font-style: normal;
  font-weight: normal;
  font-size: 1rem;
  line-height: 1.688rem;
  letter-spacing: 0.063rem;
  font-feature-settings: "zero" on;
  color: white;
  padding: 8px;
  white-space: nowrap;
  max-width: 141px;
  overflow: hidden;
  text-overflow: ellipsis;
`;

const Row = styled.div`
  display: flex;
  border-bottom: 1px solid black;
  align-items: center;
  height: 56px;
`;

const Header = styled.header`
  background-color: #2d3855;
  height: 56px;
  border-bottom: 1px solid black;
  align-items: center;
  padding-left: 10px;
  display: none;
  ${mediaQueries.small} {
    display: flex;
  }
`;

const Container = styled.div`
  width: 100%;
  border-top: 1px solid black;
  visibility: hidden;
  overflow: auto;
  ${mediaQueries.small} {
    position: fixed;
    top: 0;
    background-color: #282f45;
    height: 100%;
  }
`;
