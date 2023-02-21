import styled from "@emotion/styled";
import React, { FC, useState } from "react";
import { Label } from "./Label";
import { Icon } from "../../Icon";
interface ILabelsProps {
  position?: "left" | "right";
  className?: string;
  targetRef?: React.RefObject<HTMLDivElement>;
  armed: boolean;
  onStop: () => void;
}

export const Labels: FC<ILabelsProps> = ({
  position,
  className,
  targetRef,
  armed,
  onStop,
}) => {
  const [downKeys, setDownKeys] = useState<string[]>([]);

  if (position === undefined) false;

  return (
    <Container ref={targetRef}>
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

const Container = styled.div`
  position: absolute;
  -moz-user-select: none;
  -ms-user-select: none;
  -khtml-user-select: none;
  -webkit-user-select: none;
  -webkit-touch-callout: none;
  user-select: none;
`;
