import { Box } from "@formant/ui-sdk";
import { TopicConfiguration } from "./TopicConfiguration";
import { SectionHeader } from "./SectionHeader";
import { FC } from "react";

interface RosTopic {
  name: string;
  type: string;
  hz: number;
  enable: boolean;
}
interface ISectionProps {
  sectionName: string;
  topicList: RosTopic[];
}

export const Section: FC<ISectionProps> = ({ sectionName, topicList }) => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        justifyContent: "left",
        textAlign: "left",
      }}
    >
      <SectionHeader name={sectionName} />
      {topicList.map((_) => (
        <TopicConfiguration name={_.name} type={_.type} hz={_.hz} />
      ))}
    </Box>
  );
};
