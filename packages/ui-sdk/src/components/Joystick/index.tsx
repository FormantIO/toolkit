import styled from "@emotion/styled";
import React from "react";
import { Icon } from "../../Icon";
import border from "@images/joystick.svg";
import { Labels } from "./Labels";

export const Joystick = () => {
  return (
    <TeleopJoystick>
      <Pad>
        <Icon sx={{ height: 48, width: 48 }} name="joystick-star" />
        <Border />
        <Labels armed onStop={() => {}} />
      </Pad>
    </TeleopJoystick>
  );
};

const TeleopJoystick = styled.div`
  display: flex;
  align-items: flex-start;
  flex: 0 0 auto;
  touch-action: none;
`;

const Pad = styled.div`
  background: rgba(28, 30, 45, 0.5);
  margin: 0.625rem 0;
  height: 8.75rem;
  width: 8.75rem;
  position: relative;
  border-radius: 50%;
  user-select: none;
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
`;

const Border = styled.div`
  touch-action: none;
  pointer-events: none;
  background: url(${border}) no-repeat;
  position: absolute;
  height: 100%;
  width: 100%;
`;
