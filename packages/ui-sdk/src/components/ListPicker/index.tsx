import { ListItem } from "./ListItem";
import React, { FC, useCallback, useMemo } from "react";
import { useState } from "react";
import styles from "./index.module.scss";
import { IconInput } from "./IconButton";

interface IListPicker {
  options: string[];
  list: string[];
  setList: React.Dispatch<React.SetStateAction<string[]>>;
  maxList?: number;
}

export const ListPicker: FC<IListPicker> = ({
  options,
  list,
  maxList,
  setList,
}) => {
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
      setList((_) => (checked ? [..._, item] : _.filter((el) => el !== item)));
    },
    [list, setList]
  );

  const filteredDevices = useMemo(() => {
    if (filters.length === 0) return options;
    return options.filter((_) => _.includes(filters));
  }, [filters, options]);

  return (
    <div className={styles["list-picker"]}>
      <IconInput
        value={filters}
        onClick={handleClearFilter}
        onChange={(e: React.ChangeEvent<HTMLInputElement>) =>
          handleFilterDevices(e.target.value)
        }
      />
      {filteredDevices
        .slice(0, !!maxList ? maxList : filteredDevices.length)
        .map((_) => {
          return (
            <ListItem
              key={_}
              name={_}
              enabled={list.includes(_)}
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
