import React, { FC, useEffect, useState } from "react";
import { ITeleopJoystickConfiguration } from "./types";
import { duration, deadzone } from "./utils";
import { WindowEvent } from "./WindowEvent";
import { Timer } from "./Timer";

export interface ITeleopActionsProps {
  configuration?: ITeleopJoystickConfiguration;

  onCancelGoal?: () => void;
  onStop?: () => void;

  onLeftJoystickPressed?: () => void;
  onLeftJoystickReleased?: () => void;
  onLeftJoystickUp?: (value: number) => void;
  onLeftJoystickDown?: (value: number) => void;
  onLeftJoystickLeft?: (value: number) => void;
  onLeftJoystickRight?: (value: number) => void;
  onLeftJoystickXAxis?: (value: number) => void;
  onLeftJoystickYAxis?: (value: number) => void;

  onRightJoystickPressed?: () => void;
  onRightJoystickReleased?: () => void;
  onRightJoystickUp?: (value: number) => void;
  onRightJoystickDown?: (value: number) => void;
  onRightJoystickLeft?: (value: number) => void;
  onRightJoystickRight?: (value: number) => void;
  onRightJoystickXAxis?: (value: number) => void;
  onRightJoystickYAxis?: (value: number) => void;

  onDownKeysChanged?: (downKeys: string[]) => void;
}

export interface ITeleopActionMappings {
  [key: string]: ITeleopActionMapping;
}

export interface ITeleopActionMapping {
  key?: string;
  gamepadButtons?: number[];
  gamepadAxis?: number;
  onPressed?: () => void;
  onReleased?: () => void;
  onValue?: (value: number) => void;
}

const buttonOnOffThreshold = 0.5;
const gamepadPollingLoopInterval = duration.second / 60;

export const TeleopActions: FC<ITeleopActionsProps> = ({
  configuration,

  onCancelGoal,
  onStop,

  onLeftJoystickPressed,
  onLeftJoystickReleased,
  onLeftJoystickUp,
  onLeftJoystickDown,
  onLeftJoystickLeft,
  onLeftJoystickRight,
  onLeftJoystickXAxis,
  onLeftJoystickYAxis,

  onRightJoystickPressed,
  onRightJoystickReleased,
  onRightJoystickUp,
  onRightJoystickDown,
  onRightJoystickLeft,
  onRightJoystickRight,
  onRightJoystickXAxis,
  onRightJoystickYAxis,

  onDownKeysChanged,
}) => {
  const [mappings, setMappings] = useState<ITeleopActionMappings>({});
  const [downKeys, setDownKeys] = useState<string[]>([]);
  const [downGamepadButtons, setDownGamepadButtons] = useState<number[]>([]);
  const [localGamepadConnected, setLocalGamepadConnected] = useState(false);
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const [hasInteractedWithGamepad, setHasInteractedWithGamepad] =
    useState(false);
  const [lastGamepadButtons, setLastGamepadButtons] = useState<GamepadButton[]>(
    []
  );

  useEffect(() => {
    loadMappings();
  }, []);

  const onKeyDown = (event: KeyboardEvent) => {
    // if (event.key === undefined) {
    //     logger.error(
    //         "onKeyDown KeyboardEvent had undefined 'key' property in TeleopActions"
    //     );
    //     return;
    // }

    const key = event.key.toLowerCase();

    if (downKeys.indexOf(key) !== -1) {
      return;
    }
    setDownKeys((prev) => [...prev, key]);
    getKeyMappings(key).forEach((_) => {
      _.onValue?.(1);
      _.onPressed?.();
    });

    onDownKeysChanged?.(downKeys);
  };

  const getKeyMappings = (key: string): ITeleopActionMapping[] => {
    return (
      Object.keys(mappings).filter((_) => mappings[_]?.key === key) || []
    ).map((_) => mappings[_]);
  };

  const onBlur = (_: FocusEvent) => {
    downKeys.forEach((key) => {
      getKeyMappings(key).forEach((keyMapping) => {
        keyMapping.onValue?.(0);
        keyMapping.onReleased?.();
      });
    });
    onStop?.();
  };

  const onKeyUp = (event: KeyboardEvent) => {
    // if (event.key === undefined) {
    //     logger.error(
    //         "onKeyUp KeyboardEvent had undefined 'key' property in TeleopActions"
    //     );
    //     return;
    // }

    const key = event.key.toLowerCase();

    getKeyMappings(key).forEach((_) => {
      _.onValue?.(0);
      _.onReleased?.();
    });

    setDownKeys((prevKeys) => prevKeys.filter((_) => _ !== key));
    onDownKeysChanged?.(downKeys);
  };

  const getGamepadButtonMappings = (
    buttons: readonly GamepadButton[]
  ): ITeleopActionMapping[] => {
    return Object.keys(mappings)
      .filter((mapping) =>
        buttons.some((_, index) =>
          mappings[mapping]?.gamepadButtons?.includes(index)
        )
      )
      .map((_) => mappings[_]);
  };
  const getGamepadAxisMappings = (
    axes: readonly number[]
  ): ITeleopActionMapping[] => {
    return Object.keys(mappings)
      .filter((mapping) =>
        axes.some((_, index) => mappings[mapping]?.gamepadAxis === index)
      )
      .map((_) => mappings[_]);
  };
  const onPollGamepad = () => {
    const gamepad = navigator?.getGamepads()[0];

    if (!gamepad) {
      if (gamepadConnected) {
        setGamepadConnected(false);
      }
      if (localGamepadConnected) {
        setLocalGamepadConnected(false);
        onLeftJoystickXAxis?.(0);
        onLeftJoystickYAxis?.(0);
        onRightJoystickXAxis?.(0);
        onRightJoystickYAxis?.(0);
        onLeftJoystickDown?.(0);
        onLeftJoystickUp?.(0);
        onLeftJoystickReleased?.();
      }
      return;
    }

    if (!gamepadConnected) {
      setGamepadConnected(true);
    }
    setLocalGamepadConnected(true);

    const { buttons, axes } = gamepad;
    const gamepadButtonActions = getGamepadButtonMappings(buttons);
    gamepadButtonActions.forEach(
      ({ gamepadButtons, onPressed, onReleased, onValue }) => {
        if (gamepadButtons === undefined) {
          return;
        }
        gamepadButtons.forEach((gamepadButton) => {
          const value = deadzone(buttons[gamepadButton].value);

          if (value !== 0 && hasInteractedWithGamepad === false) {
            setHasInteractedWithGamepad(true);
          }

          const isAlreadyDown = downGamepadButtons.includes(gamepadButton);

          const lastValue = deadzone(
            lastGamepadButtons[gamepadButton]?.value || 0
          );

          if (!hasInteractedWithGamepad) {
            return;
          }

          if (value > buttonOnOffThreshold && !isAlreadyDown) {
            setDownGamepadButtons((prev) => [...prev, gamepadButton]);
            onPressed?.();
          }

          if (value <= buttonOnOffThreshold) {
            setDownGamepadButtons((prevButtons) =>
              prevButtons.filter((_) => _ !== gamepadButton)
            );
            if (isAlreadyDown) {
              onReleased?.();
            }
          }

          if (lastValue !== value) {
            onValue?.(value);
          }
        });
      }
    );
    setLastGamepadButtons([...buttons]);

    const gamepadAxisActions = getGamepadAxisMappings(axes);
    gamepadAxisActions.forEach(({ gamepadAxis, onValue }) => {
      if (gamepadAxis === undefined) {
        return;
      }

      const value = deadzone(axes[gamepadAxis]);

      if (value !== 0 && hasInteractedWithGamepad === false) {
        setHasInteractedWithGamepad(true);
      }

      if (hasInteractedWithGamepad) {
        onValue?.(value);
      }
    });
  };

  const loadMappings = () => {
    setMappings({
      cancelGoal: {
        key: "Escape",
        onPressed: () => onCancelGoal?.(),
      },
      leftJoystickUp: {
        key: "w",
        gamepadButtons: configuration?.disableTriggerJoystickMapping
          ? undefined
          : [7],
        onPressed: onLeftJoystickPressed,
        onReleased: onLeftJoystickReleased,
        onValue: onLeftJoystickUp,
      },
      leftJoystickDown: {
        key: "s",
        gamepadButtons: configuration?.disableTriggerJoystickMapping
          ? undefined
          : [6],
        onPressed: onLeftJoystickPressed,
        onReleased: onLeftJoystickReleased,
        onValue: onLeftJoystickDown,
      },
      leftJoystickLeft: {
        key: "a",
        onPressed: onLeftJoystickPressed,
        onReleased: onLeftJoystickReleased,
        onValue: onLeftJoystickLeft,
      },
      leftJoystickRight: {
        key: "d",
        onPressed: onLeftJoystickPressed,
        onReleased: onLeftJoystickReleased,
        onValue: onLeftJoystickRight,
      },
      leftJoystickXAxis: {
        gamepadAxis: configuration?.x?.gamepadAxis,
        onValue: onLeftJoystickXAxis,
      },
      leftJoystickYAxis: {
        gamepadAxis: configuration?.y?.gamepadAxis,
        onValue: onLeftJoystickYAxis,
      },
      rightJoystickUp: {
        key: "arrowup",
        onPressed: onRightJoystickPressed,
        onReleased: onRightJoystickReleased,
        onValue: onRightJoystickUp,
      },
      rightJoystickDown: {
        key: "arrowdown",
        onPressed: onRightJoystickPressed,
        onReleased: onRightJoystickReleased,
        onValue: onRightJoystickDown,
      },
      rightJoystickLeft: {
        key: "arrowleft",
        onPressed: onRightJoystickPressed,
        onReleased: onRightJoystickReleased,
        onValue: onRightJoystickLeft,
      },
      rightJoystickRight: {
        key: "arrowright",
        onPressed: onRightJoystickPressed,
        onReleased: onRightJoystickReleased,
        onValue: onRightJoystickRight,
      },
      rightJoystickXAxis: {
        gamepadAxis: configuration?.x?.gamepadAxis,
        onValue: onRightJoystickXAxis,
      },
      rightJoystickYAxis: {
        gamepadAxis: configuration?.y?.gamepadAxis,
        onValue: onRightJoystickYAxis,
      },
    });
  };

  return (
    <>
      <WindowEvent type="keydown" onEvent={onKeyDown} />
      <WindowEvent type="blur" onEvent={onBlur} />
      <WindowEvent type="keyup" onEvent={onKeyUp} />
      <Timer interval={gamepadPollingLoopInterval} onTick={onPollGamepad} />
    </>
  );
};
