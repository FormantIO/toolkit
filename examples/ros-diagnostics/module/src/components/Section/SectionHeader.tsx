import { Box, Icon, TextField } from "@formant/ui-sdk";
import React, {
  FC,
  ChangeEventHandler,
  useCallback,
  SetStateAction,
  useEffect,
} from "react";
import { updatePath } from "../updatePath";
import { get, unset } from "lodash";
import { v4 as uuidv4 } from "uuid";

interface ISectionHeader {
  path: string;
  setParams: React.Dispatch<SetStateAction<any>>;
  params: any;
  index: string;
  handleOnDelete: () => void;
}

export const SectionHeader: FC<ISectionHeader> = (props) => {
  const { setParams, path, params, index, handleOnDelete } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) => {
      setParams((prev: any) => updatePath(prev, path, e.target.value));
    },
    [path, setParams]
  );

  const handleAddTopic = useCallback(() => {
    const currentTopics = get(params, `[${index}][contents]`);

    setParams((prev: any) =>
      updatePath(prev, `[${index}][contents]`, {
        [uuidv4()]: {
          topicName: "",
          type: "",
          hz: "",
          enabled: true,
        },
        ...currentTopics,
      })
    );
  }, [path, setParams]);

  return (
    <Box
      sx={{
        display: "flex",
        width: "100%",
        alignItems: "center",
        justifyContent: "space-between",
        marginTop: 1,
      }}
    >
      <TextField
        type="text"
        sx={{ marginBottom: "16px", width: 230 }}
        label={"Section"}
        variant="standard"
        value={get(params, path) ?? ""}
        onChange={handleChange}
      />

      <Box
        sx={{
          display: "flex",
        }}
      >
        <Box
          onClick={handleAddTopic}
          sx={{
            height: 40,
            width: 40,
            borderRadius: 25,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            marginRight: 1,
            ":hover": {
              backgroundColor: "#3b4668",
              cursor: "pointer",
            },
          }}
        >
          <Icon name="plus" />
        </Box>
        {Object.keys(params).length > 1 && (
          <Box
            onClick={handleOnDelete}
            sx={{
              height: 40,
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
            <Icon name="delete" />
          </Box>
        )}
      </Box>
    </Box>
  );
};
