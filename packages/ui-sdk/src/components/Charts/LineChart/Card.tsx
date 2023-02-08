import React from "react";
import styled from "@emotion/styled";

import { Icon } from "../../../Icon";
export const Card: React.FC = () => {
  return (
    <TooltipContainer id="chartjs-tooltip">
      <TextContainer>
        <Label>
          X: <Xpoint id="tooltop-text" />,
        </Label>
        <Label>
          Y: <Ypoint id="tooltop-text-2" />
        </Label>
        <DateContainer>
          {`${new Date().toISOString().slice(11, -5)} pm`}
        </DateContainer>
      </TextContainer>
      <TextContainer style={{ marginTop: 5 }}>
        <Icon name="device" />
        <DeviceName>Astro</DeviceName>
      </TextContainer>
    </TooltipContainer>
  );
};

const TooltipContainer = styled.div`
  font-family: "inter";
  min-height: 40px;
  min-width: 180px;
  width: auto;
  background-color: black;
  border-radius: 4px;
  display: flex;
  flex-direction: column;
  padding: 15px 20px 15px 20px;
  position: relative;
  &:before {
    content: "";
    background-color: #36a0fe;
    height: 65px;
    width: 5px;
    border-top-right-radius: 25px;
    border-bottom-right-radius: 25px;
    position: absolute;
    top: 15px;
    left: 0px;
  }
  p {
    color: #fff;
    font-size: 24px;
  }
`;

const TextContainer = styled.div`
  display: flex;
`;

const Label = styled.span`
  color: white;
  font-size: 24px;
`;

const Xpoint = styled.span`
  color: #fff;
  font-size: 24px;
`;

const Ypoint = styled.span`
  color: #fff;
  font-size: 24px;
`;

const DateContainer = styled.span`
  color: white;
  font-size: 14px;
  letter-spacing: 0.4px;
  margin-left: 15px;
  margin-top: 3px;
`;

const DeviceName = styled.span`
  font-size: 18px;
  color: #bac4e2;
  margin-left: 10px;
  margin-top: 3px;
`;
