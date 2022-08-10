import { Box } from "@formant/ui-sdk";
import { TopicConfiguration } from "./TopicConfiguration";
import { SectionHeader } from "./SectionHeader";
import { FC, useEffect } from "react";
import RosTopicStats from "../../types/RosTopicStats";

interface ISectionProps {
  topicList: RosTopicStats;
  params: any;
  setParams: any;
  index: string;
}

export const Section: FC<ISectionProps> = ({
  topicList,
  params,
  setParams,
  index,
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
        params={params}
        setParams={setParams}
      />
      {Object.keys(topicList).map((_) => (
        <TopicConfiguration
          path={`[${index}][contents][${_}]`}
          params={params}
          setParams={setParams}
          key={topicList[_].name}
          name={topicList[_].name}
          type={topicList[_].type}
          hz={topicList[_].hz}
        />
      ))}
    </Box>
  );
};
