import { Box, Button, Icon } from "@formant/ui-sdk";
import { FC, useState, useLayoutEffect, useMemo } from "react";
import { JsonSchemaForm } from "./JsonSchemaForm/index";
import { KeyValue } from "@formant/data-sdk";
import { Footer } from "./Footer";
import { Header } from "./Header";
import { AddTopic } from "./AddTopic";
import { Section } from "./Section/index";
import RosTopicStats, { OnlineTopics } from "../types/RosTopicStats";
import { v4 as uuidv4 } from "uuid";

interface IModuleConfig {
  topicStats: OnlineTopics;
  closeConfig: () => void;
  showSnackBar: () => void;
  jsonObjectFromCloud: any;
  currentConfiuration?: any;
}

// interface Configuration {
//   section: string;
//   contents: RosTopicStats[];
// }

export const ModuleConfig: FC<IModuleConfig> = ({
  topicStats,
  closeConfig,
  showSnackBar,
  currentConfiuration,
}) => {
  const [schema, setSchema] = useState<any>();
  const [showAddTopic, setShowAddTopic] = useState(false);
  const [configuration, setConfguration] = useState<OnlineTopics>({
    default: {
      section: "",
      contents: { default: { topicName: "", type: "", hz: 0, enable: true } },
    },
  });

  useLayoutEffect(() => {
    if (currentConfiuration === undefined) {
      setConfguration(topicStats);
      return;
    }
    setConfguration(currentConfiuration);
  }, []);

  const saveConfiguraton = async () => {
    const err = Object.keys(configuration).filter(
      (_) => configuration[_].section.length < 1
    );
    if (err.length > 0) return; //Section name can't be empty ;
    await KeyValue.set(
      "rosDiagnosticsConfiguration",
      JSON.stringify(configuration)
    );
    closeConfig();
    showSnackBar();
  };

  return showAddTopic ? (
    <AddTopic
      currentConfiguration={currentConfiuration}
      onBack={() => setShowAddTopic(false)}
      onSave={async (_) => {
        await KeyValue.set("rosDiagnosticsConfiguration", JSON.stringify(_));
        closeConfig();
        showSnackBar();
      }}
    />
  ) : (
    <Box
      position={"relative"}
      display={"flex"}
      flexDirection="column"
      alignItems="center"
      paddingBottom={"100px"}
    >
      <Header
        onBack={closeConfig}
        buttonLabel="ADD TOPIC"
        onClick={() =>
          //ADD empty section to current configuration
          setConfguration((prev) => ({
            [uuidv4()]: {
              section: "",
              contents: {
                [uuidv4()]: { topicName: "", type: "", hz: 0, enable: true },
              },
            },
            ...prev,
          }))
        }
        label="Topics"
      />

      {Object.keys(configuration).map((_) => {
        return (
          <Section
            key={_}
            index={_}
            params={configuration}
            setParams={setConfguration}
            topicList={configuration[_].contents}
          />
        );
      })}
      <Footer
        onCancel={closeConfig}
        onClick={saveConfiguraton}
        label={"Save"}
      />
    </Box>
  );
};
