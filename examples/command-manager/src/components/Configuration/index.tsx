import { CommandSetter } from "../CommandSetter";
import { ICommand } from "../../types";
import { FC } from "react";
import { Icon } from "@formant/ui-sdk";
import styles from "./index.module.scss";
import { useSelector } from "react-redux";

interface IConfigurationProps {
  commands: ICommand[];
  handleCloseconfiguration: () => void;
}

export const Configuration: FC<IConfigurationProps> = ({
  commands,
  handleCloseconfiguration,
}) => {
  const displayCommands = useSelector(
    (state: any) => state.configuration.activeCommands
  );

  return (
    <div>
      <div onClick={handleCloseconfiguration} className={styles["back-arrow"]}>
        <Icon name="arrow-left" />
      </div>
      {commands.map((_) => (
        <CommandSetter
          key={_.id}
          commandName={_.name}
          enabled={displayCommands.includes(_.id)}
          id={_.id}
        />
      ))}
    </div>
  );
};
