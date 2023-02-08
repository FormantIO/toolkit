import styled from "@emotion/styled";
import React, { FC } from "react";

interface ITooltipProps {
  containerId: string;
  labelId: string;
  valueId: string;
}

export const Tooltip: FC<ITooltipProps> = ({
  containerId,
  labelId,
  valueId,
}) => {
  return (
    <Container id={containerId}>
      <span id={labelId}></span>
      <span id={valueId}></span>
    </Container>
  );
};

const Container = styled.div`
  height: 43px;
  background-color: black;
  display: flex;
  padding: 8px 20px;
  box-shadow: 0px 4px 4px rgba(0, 0, 0, 0.5);
  border-radius: 4px;
  span {
    color: white;
  }
`;
