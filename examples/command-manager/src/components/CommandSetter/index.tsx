import { Typography, Switch } from "@formant/ui-sdk";
import { FC, useCallback } from "react";
import styles from "./index.module.scss";
import { useDispatch } from "react-redux";
import { handleCommandChange } from "../../features/configuration/configurationSlice";

interface IcomandSetterProps {
  commandName: string;
  enabled: boolean;
  id: string;
}

export const CommandSetter: FC<IcomandSetterProps> = ({
  commandName,
  enabled,
  id,
}) => {
  const dispatch = useDispatch();
  const handleOnChange = useCallback((checked: boolean) => {
    dispatch(handleCommandChange({ id: id, checked: checked }));
  }, []);

  return (
    <div className={styles["command-setter-container"]}>
      <div>
        <Typography>{commandName}</Typography>
      </div>
      <Switch
        onChange={(ev) => handleOnChange(ev.target.checked)}
        size="small"
        checked={enabled}
      />
    </div>
  );
};
