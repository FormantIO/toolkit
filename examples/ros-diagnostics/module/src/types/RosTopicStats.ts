export default interface RosTopicStats {
  [key: string]: {
    topicName: string;
    type: string;
    hz: number;
    enabled?: boolean;
  };
}

export interface OnlineTopics {
  [key: string]: {
    section: string;
    contents: RosTopicStats;
  };
}
