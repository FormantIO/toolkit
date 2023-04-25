import { useDevices } from "./useDevices";
import { DeviceSwitch } from "./DeviceSwitch";
import { FC, useCallback, useEffect, useMemo } from "react";
import { useState, useReducer } from "react";
import styles from "./index.module.scss";
import { IconInput } from "./IconInput";
import { Authentication } from "@formant/data-sdk";

interface IListPicker {
  options: string[];
  enabled: string[];
  setEnabled: React.Dispatch<React.SetStateAction<string[]>>;
  maxList?: number;
}

export const DevicePicker: FC<IListPicker> = ({
  options,
  enabled,
  maxList,
  setEnabled,
}) => {
  useEffect(() => {
    Authentication.waitTilAuthenticated().then((_) => {
      Authentication.listenForRefresh();
    });
  }, []);
  const [filters, setFilters] = useState<string>("");

  const handleFilterDevices = useCallback((value: string | null) => {
    if (value === null) {
      setFilters("");
      return;
    }
    setFilters(value);
  }, []);

  const handleClearFilter = useCallback(() => {
    setFilters("");
  }, []);

  const handleChangeSwitch = useCallback(
    (checked: boolean, item: string) => {
      setEnabled((_) =>
        checked ? [..._, item] : _.filter((el) => el !== item)
      );
    },
    [enabled, setEnabled]
  );

  const filteredDevices = useMemo(() => {
    if (filters.length === 0) return options;
    return options.filter((_) => _.includes(filters));
  }, [filters, options]);

  return (
    <div className={styles["device-picker"]}>
      <IconInput
        value={filters}
        onClear={handleClearFilter}
        onChange={(e: React.ChangeEvent<HTMLInputElement>) =>
          handleFilterDevices(e.target.value)
        }
      />
      {filteredDevices
        .slice(0, !!maxList ? maxList : filteredDevices.length)
        .map((_) => {
          return (
            <DeviceSwitch
              key={_}
              name={_}
              enabled={enabled.includes(_)}
              onChange={(
                e: React.ChangeEvent<HTMLInputElement>,
                checked: boolean
              ) => handleChangeSwitch(checked, _)}
            />
          );
        })}
    </div>
  );
};
