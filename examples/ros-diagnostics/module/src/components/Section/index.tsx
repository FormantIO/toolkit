import { Box } from "@formant/ui-sdk";
import { TopicConfiguration } from "./TopicConfiguration";
import { SectionHeader } from "./SectionHeader";
import { FC, useEffect } from "react";
import RosTopicStats from "../../types/RosTopicStats";
import { Options } from "./Options";

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
      />
      {Object.keys(topicList).map((_) => (
        <TopicConfiguration
          path={`[${index}][contents][${_}]`}
          params={params}
          setParams={setParams}
          key={`[${index}][contents][${_}]`}
          name={params[index].section}
          handleOpenOptions={handleOpenOptions}
        />
      ))}
    </Box>
  );
};
