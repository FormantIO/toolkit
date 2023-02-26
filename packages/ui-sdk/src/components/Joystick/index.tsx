import styled from "@emotion/styled";
import React, { createRef, FC, useEffect, useRef, useState } from "react";
import { Icon } from "../../Icon";
import border from "@images/joystick.svg";
import { Labels } from "./Labels";
import { css } from "@emotion/css";
import classNames from "classnames";
import {
  ITeleopJoystickConfiguration,
  ITeleopTwistValue,
  IVector2,
} from "./types";

import {
  clamp,
  clampVectorLength,
  deadzone,
  debounce,
  range,
  defaultNewValueJoystickDebounce,
  defaultSameValueJoystickDebounce,
  duration,
  addVectors,
  vectorLength,
} from "./utils";
import { TeleopActions } from "./TeleopActions";
import { Timer } from "./Timer";
import { WindowEvent } from "./WindowEvent";

const dotSize = 1.875;
const center = { x: 0, y: 0 };
interface IJoystickProps {
  joystickConfiguration: ITeleopJoystickConfiguration;
  armed: boolean;
  onSendTwistValues: (
    twistValues: ITeleopTwistValue[],
    reliable: boolean
  ) => void;
}

export const Joystick: FC<IJoystickProps> = ({
  joystickConfiguration,
  armed,
  onSendTwistValues,
}) => {
  const pad = useRef<HTMLDivElement>();
  const label: React.RefObject<HTMLDivElement> = createRef();
  const [joystickCoordinates, setJoystickCoordinates] = useState<IVector2>();
  const [dotCoordinates, setDotCoordinates] = useState<IVector2>();
  const [mouseDown, setMouseDown] = useState<boolean>(false);
  const [startTarget, setStartTarget] = useState<EventTarget>();
  const [lastValues, setLastValues] = useState<ITeleopTwistValue[]>([]);
  const [canSendSameValue, setCanSendSameValue] = useState(true);
  const [canSendNewValue, setCanSendNewValue] = useState(true);
  const [tryingToInteract, setTryingToInteract] = useState(false);

  const [up, setUp] = useState<IVector2>({ x: 0, y: 0 });
  const [down, setDown] = useState<IVector2>({ x: 0, y: 0 });
  const [left, setLeft] = useState<IVector2>({ x: 0, y: 0 });
  const [right, setRight] = useState<IVector2>({ x: 0, y: 0 });
  const [xAxis, setXAxis] = useState<IVector2>({ x: 0, y: 0 });
  const [yAxis, setYAxis] = useState<IVector2>({ x: 0, y: 0 });
  const [active, setActive] = useState(false);

  useEffect(() => {
    pad.current?.addEventListener("contextmenu", (e) => {
      e.preventDefault();
    });
  }, []);
  const updateJoystickCoordinates = (x: number, y: number) => {
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
    setMouseDown(true);
    setStartTarget(target);
    updateJoystickCoordinates(x, y);
    sendCommand({
      coordinates: joystickCoordinates,
      reliable: false,
    });
  };

  const onTouchStart = (event: React.TouchEvent<HTMLDivElement>) => {
    const { touches } = event;
    const touchList = range(0, touches.length)
      .map((_) => touches.item(_))
      .filter((_) => _?.target === pad.current || _?.target === label.current);

    if (!touchList[0]) {
      return;
    }
    const { clientX: x, clientY: y, target } = touchList[0];
    setMouseDown(true);
    setStartTarget(target);
    updateJoystickCoordinates(x, y);
    // this.sendCommand({
    //   coordinates: this.joystickCoordinates,
    //   reliable: false,
    // });
  };

  const setDotFromJoystickCoordinates = (joystickCoords: IVector2) => {
    if (!pad.current) {
      return;
    }
    const { height, width } = pad.current.getBoundingClientRect();

    const { x, y } = clampVectorLength(joystickCoords, 0.8);

    setDotCoordinates({
      x: ((x + 1) / 2) * width,
      y: ((y - 1) / -2) * height,
    });
  };

  const sendCommand = (config: {
    coordinates?: IVector2;
    reliable: boolean;
  }) => {
    const { coordinates, reliable } = config;
    const { x, y } = coordinates || { x: 0, y: 0 };
    const outputDimensions: ITeleopTwistValue[] = [];
    // const { joystick: ftue } = ftueStore;
    if (!joystickConfiguration) {
      return;
    }
    // if (ftue) {
    //   ftueStore.joystick = false;
    // }

    const { x: xAxis, y: yAxis } = joystickConfiguration;

    if (xAxis?.dimension) {
      outputDimensions.push({
        dimension: xAxis?.dimension,
        value:
          Math.abs(Math.pow(Math.abs(x), xAxis.expo || 2)) *
            (xAxis.scale || 1) *
            Math.sign(x) || 0,
      });
    }

    if (yAxis?.dimension) {
      outputDimensions.push({
        dimension: yAxis.dimension,
        value:
          Math.abs(Math.pow(Math.abs(y), yAxis.expo || 2)) *
            (yAxis.scale || 1) *
            Math.sign(y) || 0,
      });
    }

    if (joystickCoordinates && !mouseDown) {
      setDotFromJoystickCoordinates(joystickCoordinates);
    }

    const equivalent = lastValues === outputDimensions;
    if (
      !reliable &&
      ((equivalent && !canSendSameValue) || (!equivalent && !canSendNewValue))
    ) {
      return;
    }

    setCanSendSameValue(false);
    setCanSendNewValue(false);

    newValueCooldown();
    sameValueCooldown();
    setLastValues(outputDimensions);
    onSendTwistValues(outputDimensions, reliable);
  };

  const sameValueCooldown = debounce(
    () => setCanSendSameValue(true),
    (joystickConfiguration.sameValueDebounce ??
      defaultSameValueJoystickDebounce) * duration.second
  );
  const newValueCooldown = debounce(
    () => setCanSendNewValue(true),
    (joystickConfiguration.newValueDebounce ??
      defaultNewValueJoystickDebounce) * duration.second
  );
  const onTryToInteract = () => {
    setTryingToInteract(true);

    setTimeout(() => {
      setTryingToInteract(false);
    }, 300);
  };

  const onUp = (value: number) => {
    setUp((prev) => ({ ...prev, y: value }));
    if (!armed) {
      onTryToInteract();
    }
  };

  const onDown = (value: number) => {
    setDown((prev) => ({ ...prev, y: -value }));

    if (!armed) {
      setTryingToInteract(true);
      onTryToInteract();
    }
  };

  const onLeft = (value: number) => {
    setLeft((prev) => ({ ...prev, x: -value }));

    if (!armed) {
      setTryingToInteract(true);
      onTryToInteract();
    }
  };

  const onRight = (value: number) => {
    setRight((prev) => ({ ...prev, x: value }));
    if (!armed) {
      setTryingToInteract(true);
      onTryToInteract();
    }
  };

  const addInputs = (reliable: boolean) => {
    if (mouseDown) {
      return;
    }

    const coords = armed
      ? [up, down, left, right, xAxis, yAxis].reduce((agg, _) =>
          addVectors(agg, _)
        )
      : { x: 0, y: 0 };

    setJoystickCoordinates(coords);

    setActive(vectorLength(coords) > 0);

    sendCommand({
      coordinates: joystickCoordinates,
      reliable,
    });
  };

  const onJoystickPressed = () => {
    addInputs(true);
  };

  const onJoystickReleased = () => {
    addInputs(true);
  };

  const onXAxis = (value: number) => {
    setXAxis((prev) => ({ ...prev, x: clamp(deadzone(value)) }));
    addInputs(false);
  };

  const onYAxis = (value: number) => {
    setYAxis((prev) => ({ ...prev, y: clamp(deadzone(value)) * -1 }));
    addInputs(false);
  };

  const onStop = () => {
    setJoystickCoordinates({ ...center });
    setActive(false);
    sendCommand({
      coordinates: center,
      reliable: false,
    });
  };

  const onTick = () => {
    if (mouseDown || active) {
      sendCommand({
        coordinates: joystickCoordinates,
        reliable: false,
      });
    }
  };

  const onTouchMove = (event: TouchEvent) => {
    if (event.target !== startTarget) {
      return;
    }

    const { touches } = event;
    const touchList = range(0, touches.length)
      .map((_) => touches.item(_))
      .filter((_) => _?.target === pad.current || _?.target === label.current);

    if (!touchList[0]) {
      return;
    }
    const { clientX: x, clientY: y } = touchList[0];
    updateJoystickCoordinates(x, y);
  };

  const onMouseUp = () => {
    setMouseDown(false);
    setJoystickCoordinates({ ...center });
    if (startTarget) {
      sendCommand({
        coordinates: center,
        reliable: true,
      });
    }
    setStartTarget(undefined);
  };

  const onMouseMove = (event: MouseEvent) => {
    const { clientX: x, clientY: y } = event;
    if (event.button !== 0 || !mouseDown) {
      return;
    }
    updateJoystickCoordinates(x, y);
  };

  const onTouchEnd = (event: TouchEvent) => {
    if (event.target !== startTarget) {
      return;
    }
    setMouseDown(false);
    setJoystickCoordinates({ ...center });
    sendCommand({
      coordinates: center,
      reliable: true,
    });

    setStartTarget(undefined);
  };

  return (
    <TeleopJoystick>
      {joystickConfiguration.position === "left" && (
        <TeleopActions
          configuration={joystickConfiguration}
          onLeftJoystickUp={onUp}
          onLeftJoystickDown={onDown}
          onLeftJoystickLeft={onLeft}
          onLeftJoystickRight={onRight}
          onLeftJoystickPressed={onJoystickPressed}
          onLeftJoystickReleased={onJoystickReleased}
          onLeftJoystickXAxis={onXAxis}
          onLeftJoystickYAxis={onYAxis}
          onStop={onStop}
        />
      )}
      {joystickConfiguration.position === "right" && (
        <TeleopActions
          configuration={joystickConfiguration}
          onRightJoystickUp={onUp}
          onRightJoystickDown={onDown}
          onRightJoystickLeft={onLeft}
          onRightJoystickRight={onRight}
          onRightJoystickPressed={onJoystickPressed}
          onRightJoystickReleased={onJoystickReleased}
          onRightJoystickXAxis={onXAxis}
          onRightJoystickYAxis={onYAxis}
          onStop={onStop}
        />
      )}
      <WindowEvent
        type="touchmove"
        onEvent={onTouchMove}
        options={{ passive: false }}
      />
      <WindowEvent type="mouseup" onEvent={onMouseUp} />
      <WindowEvent type="mousemove" onEvent={onMouseMove} />
      <WindowEvent type="touchend" onEvent={onTouchEnd} />
      <Pad
        onMouseDown={onMouseDown}
        onTouchStart={onTouchStart}
        ref={(_) => (pad.current = _ || undefined)}
        className={classNames(mouseDown && isActive)}
      >
        <Timer interval={duration.second * 0.05} onTick={onTick} />
        <Star name="joystick-star" />
        <Border mousedown={mouseDown} />
        <Labels
          targetRef={label}
          mouseDown={mouseDown}
          position="right"
          armed
          onStop={onStop}
        />
        <Dot
          mousedown={mouseDown}
          style={{
            top: dotCoordinates?.y,
            left: dotCoordinates?.x,
          }}
        />
      </Pad>
    </TeleopJoystick>
  );
};

interface IActiveProps {
  mousedown: boolean;
}

const Star = styled(Icon)`
  height: 48px;
  width: 48px;
  opacity: 0.1;
  transition: inherit;

  &:hover {
    opacity: 0.25;
  }
`;

const isActive = css`
  background: rgba(black, 0.5);
  cursor: pointer;
  opacity: 1 !important;
`;

const Dot = styled.div<IActiveProps>`
  touch-action: none;
  pointer-events: none;
  position: absolute;
  height: ${dotSize}rem;
  width: ${dotSize}rem;
  border-radius: 50%;
  background: white;
  opacity: ${(props: any) => (props.mousedown ? 1 : 0)};
  margin: -${dotSize * 0.5}rem 0 0 -${dotSize * 0.5}rem;
  z-index: 2;
  top: 0;
  left: 0;
`;

const TeleopJoystick = styled.div`
  display: flex;
  align-items: flex-start;
  touch-action: none;
  transition: all ease-in-out 200ms;
`;

const Border = styled.div<IActiveProps>`
  touch-action: none;
  pointer-events: none;
  background: url(${border}) no-repeat;
  position: absolute;
  height: 100%;
  width: 100%;
  opacity: ${(props) => (props.mousedown ? 1 : 0.25)};
  transition: inherit;
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
  transition: all ease-in-out 0.2s;
  &:hover {
    transition: all ease-in-out 0.2s;
    cursor: pointer;
    ${Star}, ${Border} {
      opacity: 1;
    }
  }
`;
