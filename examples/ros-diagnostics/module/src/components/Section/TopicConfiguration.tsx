import { Box, TextField, Icon } from "@formant/ui-sdk";
import {
  FC,
  useCallback,
  ChangeEventHandler,
  MouseEvent,
  useEffect,
} from "react";
import { updatePath } from "../updatePath";
import { get } from "lodash";
import { Options } from "./Options";
interface ITopicConfigurationProps {
  name: string;
  setParams: any;
  path: any;
  params: any;
  handleOpenOptions: (
    e: React.MouseEvent<HTMLDivElement, globalThis.MouseEvent>,
    _: string,
    _path: string[]
  ) => void;
}

export const TopicConfiguration: FC<ITopicConfigurationProps> = (props) => {
  const { setParams, path, params, name, handleOpenOptions } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) => {
      setParams((prev: any) =>
        //use 'name' property in the 'event' object to extend the path
        updatePath(prev, path + `[${e.target.name}]`, e.target.value)
      );
    },
    [path]
  );

  return (
    <Box
      sx={{
        display: "flex",
        minWidth: 700,
        justifyContent: "space-between",
        alignItems: "center",
        marginBottom: 1.6,
        height: 57,
      }}
      display={"flex"}
    >
      <TextField
        sx={{ marginRight: 1, width: "45%" }}
        type="text"
        label={"Topic"}
        variant="filled"
        name="topicName"
        value={get(params, path + "[topicName]") ?? ""}
        onChange={handleChange}
      />

      <TextField
        sx={{ marginRight: 1, width: "45%" }}
        type="text"
        label={"Type"}
        variant="filled"
        name="type"
        value={get(params, path + "[type]") ?? ""}
        onChange={handleChange}
      />
      <TextField
        sx={{ marginRight: 1, width: 100 }}
        type="number"
        label={"Hz"}
        variant="filled"
        name="hz"
        value={Math.floor(get(params, path + "[hz]")) ?? ""}
        onChange={handleChange}
      />
      <Box
        onClick={(e) => handleOpenOptions(e, name, path)}
        sx={{
          height: 35,
          width: 40,
          borderRadius: 25,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          ":hover": {
            backgroundColor: "#3b4668",
            cursor: "pointer",
          },
        }}
      >
        <Icon name="more" />
      </Box>
    </Box>
  );
};
