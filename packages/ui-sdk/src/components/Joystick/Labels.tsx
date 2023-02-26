import styled from "@emotion/styled";
import React, { FC, useState } from "react";
import { Label } from "./Label";
import { Icon } from "../../Icon";
interface ILabelsProps {
  position?: "left" | "right";
  mouseDown: boolean;
  targetRef?: React.RefObject<HTMLDivElement>;
  armed: boolean;
  onStop: () => void;
}

export const Labels: FC<ILabelsProps> = ({
  position,
  targetRef,
  armed,
  onStop,
  mouseDown,
}) => {
  const [downKeys, setDownKeys] = useState<string[]>([]);

  if (position === undefined) false;

  return (
    <Container mouseDown={mouseDown} ref={targetRef}>
      {position === "right" && (
        <>
          <Label position="top" active={armed && downKeys.includes("arrowup")}>
            <Icon name="arrow-up" sx={{ height: 18, width: 18 }} />
          </Label>
          <Label
            position="right"
            active={armed && downKeys.includes("arrowright")}
          >
            <Icon name="arrow-right" sx={{ height: 18, width: 18 }} />
          </Label>
          <Label
            position="bottom"
            active={armed && downKeys.includes("arrowdown")}
          >
            <Icon name="arrow-down" sx={{ height: 18, width: 18 }} />
          </Label>
          <Label
            position="left"
            active={armed && downKeys.includes("arrowleft")}
          >
            <Icon name="arrow-left" sx={{ height: 18, width: 18 }} />
          </Label>
        </>
      )}
      {position === "left" && (
        <>
          <Label position="top" active={armed && downKeys.includes("w")}>
            W
          </Label>
          <Label position="right" active={armed && downKeys.includes("d")}>
            D
          </Label>
          <Label position="bottom" active={armed && downKeys.includes("s")}>
            S
          </Label>
          <Label position="left" active={armed && downKeys.includes("a")}>
            A
          </Label>
        </>
      )}
    </Container>
  );
};

interface IActiveProps {
  mouseDown: boolean;
}

const Container = styled.div<IActiveProps>`
  position: absolute;
  top: 0;
  bottom: 0;
  left: 0;
  right; 0;
  box-sizing: border-box;
  width: 100%;
  height: 100%
  -moz-user-select: none;
  -ms-user-select: none;
  -khtml-user-select: none;
  -webkit-user-select: none;
  -webkit-touch-callout: none;
  user-select: none;
  opacity: ${(props) => (props.mouseDown ? 1 : 0.5)}
  &:hover {
    opacity: 1
  }
`;
