import styles from "./DeviceSwitch.module.scss";
import { Switch, Typography, TextField } from "@formant/ui-sdk";
import { Autocomplete } from "@mui/material";
import { FC } from "react";

interface IDeviceSwitchProps {
  name: string;
  enabled: boolean;
  onChange: (
    event: React.ChangeEvent<HTMLInputElement>,
    checked: boolean
  ) => void;
}

export const DeviceSwitch: FC<IDeviceSwitchProps> = ({
  name,
  enabled,
  onChange,
}) => {
  return (
    <div className={styles["device-switch"]}>
      
      <Typography variant="body2">{name}</Typography>{" "}
      <Switch size="small" onChange={onChange} checked={enabled} value={enabled} />
    </div>
  );
};
