import { Box } from "@formant/ui-sdk";
import { TopicConfiguration } from "./TopicConfiguration";
import { SectionHeader } from "./SectionHeader";
import { FC, useEffect, useMemo, useCallback, useState } from "react";
import RosTopicStats from "../../types/RosTopicStats";
import { Options } from "./Options";
import { DialogComponent } from "../DialogComponent";
import { unset, get } from "lodash";
interface ISectionProps {
  topicList: RosTopicStats;
  params: any;
  setParams: any;
  index: string;
  handleOpenOptions: (
    e: React.MouseEvent<HTMLDivElement, globalThis.MouseEvent>,
    _: string,
    path: string[]
  ) => void;
}

export const Section: FC<ISectionProps> = ({
  topicList,
  params,
  setParams,
  index,
  handleOpenOptions,
}) => {
  const [open, setOpen] = useState(false);
  const handleDeleteSection = useCallback(() => {
    const deepCopy = JSON.parse(JSON.stringify(params));
    unset(deepCopy, index);
    setParams(deepCopy);
  }, [params]);

  const handleOpenDialog = useCallback(() => {
    setOpen(true);
  }, []);

  const handleCloseDialog = useCallback(() => {
    setOpen(false);
  }, []);

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        justifyContent: "left",
        textAlign: "left",
      }}
    >
      <SectionHeader
        path={`[${index}][section]`}
        index={index}
        params={params}
        setParams={setParams}
        handleOnDelete={handleOpenDialog}
      />
      {Object.keys(topicList).map((_) => (
        <TopicConfiguration
          path={`[${index}][contents][${_}]`}
          params={params}
          setParams={setParams}
          key={`[${index}][contents][${_}]`}
          name={params[index].section}
          handleOpenOptions={handleOpenOptions}
          enabled={topicList[_].enabled ?? true}
        />
      ))}
      <DialogComponent
        openDialog={open}
        title={"Delete Topic"}
        handleCloseDialog={handleCloseDialog}
        description={`Delete section "${get(
          params,
          `[${index}][section]`
        )}" and its topics ?`}
        onOk={handleDeleteSection}
      />
    </Box>
  );
};
