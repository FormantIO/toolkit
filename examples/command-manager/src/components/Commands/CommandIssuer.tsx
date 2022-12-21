import { Button } from "@formant/ui-sdk";
import { FC, useCallback, useState } from "react";
import { ICommandConfiguration } from "../../types";
import { useDispatch } from "react-redux";
import { setModalState } from "../../features/modal/modalSlice";

interface ICommandIssuer {
  command: ICommandConfiguration;
  onClick: (_: string, p: string) => void;
}

export const CommandIssuer: FC<ICommandIssuer> = ({ command, onClick }) => {
  const dispatch = useDispatch();
  const [disabled, setDisabled] = useState(false);

  const handleOnclick = useCallback(() => {
    setDisabled(true);
    if (command.enabledParameters || command.needsConfirmation) {
      dispatch(
        setModalState({
          state: {
            title: command.name,
            open: true,
            content: `issue command ${command.name} ?`,
            streamName: command.streamName.length > 0 ? command.streamName : "",
            paramType: command.enabledParameters
              ? command.streamName.length > 0
                ? "dropDown"
                : "textField"
              : command.parameterValue,
          },
        })
      );
    } else {
      onClick(command.name, command.parameterValue ?? "");
    }
    setTimeout(() => {
      setDisabled(false);
    }, 5000);
  }, []);

  return (
    <Button
      onClick={handleOnclick}
      color="primary"
      variant="contained"
      disabled={disabled}
      sx={{
        marginLeft: 1,
        marginRight: 1,
      }}
    >
      {command.name}
    </Button>
  );
};
