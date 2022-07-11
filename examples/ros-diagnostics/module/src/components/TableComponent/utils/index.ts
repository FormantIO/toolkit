import RosTopicStats from "../../../types/RosTopicStats";

type Topic = {
  topic: string;
  type: string;
  minHz: string;
};

export const minHzForTopic = (
  topicName: string,
  cloudConfig: {
    [key: string]: {
      title: string;
      contents: Topic[];
    };
  }
) => {
  return Object.keys(cloudConfig!).reduce(
    (acc, section) =>
      Math.max(
        acc,
        cloudConfig![section].contents.reduce(
          (a, _) =>
            _.topic === topicName
              ? Math.max(a, parseInt(_.minHz))
              : Math.max(a, 0),
          0
        )
      ),
    0
  );
};

export const createJsonSchemaObjectFromConfig = (config: any) => {
  const rosDiagnosticsConfiguration = {
    type: "object",
    title: "Ros Diagnostics Configuration",
    properties: {},
  };
  Object.keys(config).forEach((_) => {
    (rosDiagnosticsConfiguration as any).properties[_] = {
      type: "object",
      title: _,
      properties: {
        section: {
          type: "string",
          title: "section",
          default: config[_].section,
        },
        type: {
          type: "string",
          title: "type",
          default: config[_].type,
        },
        minHz: {
          type: "integer",
          title: "minHz",
          default: config[_].minHz,
        },
      },
    };
  });
  return rosDiagnosticsConfiguration;
};

export const splitTopicStatsByConfig = (
  topicStats: RosTopicStats[],
  cloudConfig: {
    [key: string]: {
      title: string;
      contents: Topic[];
    };
  }
): { title: string; contents: RosTopicStats[] }[] => {
  const remainingTopics = [...topicStats];

  if (Object.keys(cloudConfig).length === 1) {
    return [{ title: "Other", contents: topicStats }];
  }

  const topicsSplitBySection = Object.keys(cloudConfig!).map((section) => {
    return {
      title: section,
      contents: cloudConfig![section].contents.map((content) => {
        const foundIndex = remainingTopics.findIndex(
          (_) => _.name === content.topic
        );
        const foundTopic =
          foundIndex >= 0
            ? remainingTopics.splice(foundIndex, 1)[0]
            : { name: content.topic, type: content.type ?? "unknown", hz: 0 };
        return foundTopic;
      }),
    };
  });
  topicsSplitBySection.push({ title: "Other", contents: remainingTopics });

  return topicsSplitBySection;
};

export const splitTopicsForSecction = (topics: {
  [key: string]: {
    section: string;
    type: string;
    minHz: string;
  };
}) => {
  const sections: {
    [key: string]: {
      title: string;
      contents: Topic[];
    };
  } = {};

  Object.keys(topics).forEach((_) => {
    if (topics[_].section === undefined || topics[_].section === "") {
      topics[_].section = "Other";
    }
    topics[_].section in sections
      ? (sections[topics[_].section] = {
          title: topics[_].section,
          contents: [
            ...sections[topics[_].section].contents,
            { topic: _, type: topics[_].type, minHz: topics[_].minHz },
          ],
        })
      : (sections[topics[_].section] = {
          title: topics[_].section,
          contents: [
            { topic: _, type: topics[_].type, minHz: topics[_].minHz },
          ],
        });
  });

  return sections;
};
