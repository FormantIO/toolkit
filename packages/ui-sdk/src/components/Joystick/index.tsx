import styled from "@emotion/styled";
import React, { useEffect, useRef, useState } from "react";
import { Icon } from "../../Icon";
import border from "@images/joystick.svg";
import { Labels } from "./Labels";
interface IVector2 {
  x: number;
  y: number;
}

const dotSize = "1.875rem";

const deadzone = (value: number, threshold = 0.1) => {
  const deadzoneValue =
    Math.abs(value) < threshold
      ? 0
      : value < 0
      ? value + threshold
      : value - threshold;
  return deadzoneValue * (1 / (1 - threshold));
};

function vectorLength(v: IVector2) {
  const { x, y } = v;
  return Math.sqrt(x * x + y * y);
}

function clampVectorLength(v: IVector2, l: number) {
  const length = vectorLength(v);
  const { x, y } = v;
  if (length > l) {
    return { x: (x / length) * l, y: (y / length) * l };
  } else {
    return v;
  }
}

export function clamp(value: number, min = -1, max = 1) {
  return value < min ? min : value > max ? max : value;
}

export const Joystick = () => {
  const pad = useRef<HTMLDivElement>();
  const [joystickCoordinates, setJoystickCoordinates] = useState<IVector2>();
  const [dotCoordinates, setDotCoordinates] = useState<IVector2>();

  useEffect(() => {
    pad.current?.addEventListener("contextmenu", (e) => {
      e.preventDefault();
    });
  }, []);
  const updateJoystickCoordinates = (x: number, y: number) => {
    // const { armed } = this.props;
    const armed = true;
    if (!pad.current) {
      return;
    }

    const { top, left, height, width } = pad.current.getBoundingClientRect();

    const coords: IVector2 = armed
      ? {
          x: clamp(((x - left) / width) * 2 - 1),
          y: clamp(((y - top) / height) * -2 + 1),
        }
      : { x: 0, y: 0 };

    setJoystickCoordinates({
      x: clamp(deadzone(coords.x)),
      y: clamp(deadzone(coords.y)),
    });

    const { x: clampedX, y: clampedY } = clampVectorLength(coords, 0.8);

    setDotCoordinates({
      x: ((clampedX + 1) / 2) * width,
      y: ((clampedY - 1) / -2) * height,
    });
  };

  const onMouseDown = (event: React.MouseEvent<HTMLDivElement>) => {
    const { clientX: x, clientY: y, target } = event;
    if (event.button !== 0) {
      event.preventDefault();
      event.stopPropagation();
      return;
    }
    // this.mouseDown = true;
    // this.startTarget = target;
    updateJoystickCoordinates(x, y);
    // this.sendCommand({
    //     coordinates: this.joystickCoordinates,
    //     reliable: false
    // });
  };

  return (
    <TeleopJoystick>
      <Pad
        onMouseDown={onMouseDown}
        ref={(_) => (pad.current = _ || undefined)}
      >
        <Icon sx={{ height: 48, width: 48 }} name="joystick-star" />
        <Border />
        <Labels position="right" armed onStop={() => {}} />
        <Dot
          style={{
            top: dotCoordinates?.y,
            left: dotCoordinates?.x,
          }}
        />
      </Pad>
    </TeleopJoystick>
  );
};

const Dot = styled.div`
  touch-action: none;
  pointer-events: none;
  position: absolute;
  height: ${dotSize};
  width: ${dotSize};
  border-radius: 50%;
  background: white;
  opacity: 1;
  margin: (-${dotSize} * 0.5) 0 0 (-${dotSize} * 0.5);
  z-index: 2;
  top: 0;
  left: 0;
`;

const TeleopJoystick = styled.div`
  display: flex;
  align-items: flex-start;
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
`;

const Border = styled.div`
  touch-action: none;
  pointer-events: none;
  background: url(${border}) no-repeat;
  position: absolute;
  height: 100%;
  width: 100%;
`;
