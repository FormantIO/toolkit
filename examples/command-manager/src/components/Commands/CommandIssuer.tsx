import { Button } from "@formant/ui-sdk";
import { FC, useCallback, useState } from "react";
import { ICommand } from "../../types";
interface ICommandIssuer {
  command: ICommand;
  handleIssueCommand: (_: string, value: string | null) => void;
}

export const CommandIssuer: FC<ICommandIssuer> = ({
  command,
  handleIssueCommand,
}) => {
  const [disabled, setDisabled] = useState(false);

  const handleOnclick = useCallback(() => {
    setDisabled(true);
    handleIssueCommand(command.name, command.parameterValue);
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
