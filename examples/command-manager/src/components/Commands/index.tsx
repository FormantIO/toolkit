import { Button, Icon, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import styles from "./index.module.scss";
import { ICommand } from "../../types";
import { CommandIssuer } from "./CommandIssuer";

interface ICommadsProps {
  commands: ICommand[];
  openConfiguration: () => void;
  handleIssueCommand: (_: string, value: string | null) => void;
}

export const Commands: FC<ICommadsProps> = ({
  commands,
  openConfiguration,
  handleIssueCommand,
}) => {
  return (
    <div className={styles.commands}>
      <div onClick={openConfiguration} className={styles.gear}>
        <Icon name="settings" />
      </div>
      {commands.length === 0 && (
        <div>
          <Typography>Click the gear to start adding commands</Typography>
        </div>
      )}
      {commands.map((_) => (
        <CommandIssuer
          key={_.id}
          handleIssueCommand={handleIssueCommand}
          command={_}
        />
      ))}
    </div>
  );
};
