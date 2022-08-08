import { Box, Button, Icon } from "@formant/ui-sdk";
import { FC, useState, useLayoutEffect, useMemo } from "react";
import { JsonSchemaForm } from "./JsonSchemaForm/index";
import { KeyValue } from "@formant/data-sdk";
import { Footer } from "./Footer";
import { Header } from "./Header";
import { AddTopic } from "./AddTopic";
import { Section } from "./Section/index";
import RosTopicStats from "../types/RosTopicStats";
interface IModuleConfig {
  topicStats: any;
  closeConfig: () => void;
  showSnackBar: () => void;
  jsonObjectFromCloud: any;
  currentConfiuration?: any;
}

interface Configuration {
  section: string;
  contents: RosTopicStats[];
}

export const ModuleConfig: FC<IModuleConfig> = ({
  topicStats,
  closeConfig,
  showSnackBar,
  jsonObjectFromCloud,
  currentConfiuration,
}) => {
  const [schema, setSchema] = useState<any>();
  const [showAddTopic, setShowAddTopic] = useState(false);
  const [configuration, setConfguration] = useState<
    { section: string; contents: RosTopicStats[] }[]
  >([
    {
      section: "",
      contents: [{ name: "", type: "", hz: 0, enable: true }],
    },
  ]);
  const config = { current: {} };

  const rosDiagnosticsConfiguration = {
    type: "object",
    title: "Ros Diagnostics Configuration",
    properties: {},
  };

  //this use effect handles render when no configuration has been created
  useLayoutEffect(() => {
    if (currentConfiuration === undefined) {
      setConfguration(topicStats);
    }
    if (jsonObjectFromCloud) return;
    //Organize topics when configuration does not exist
    // topicStats["default"].forEach((_: any) => {
    //   (rosDiagnosticsConfiguration as any).properties[_.name] = {
    //     type: "object",
    //     title: _.name,
    //     properties: {
    //       section: {
    //         type: "string",
    //         title: "section",
    //       },
    //       type: {
    //         type: "string",
    //         title: "type",
    //         default: _.type,
    //       },
    //       minHz: {
    //         type: "integer",
    //         title: "minHz",
    //         default: 0,
    //       },
    //     },
    //   };
    // });

    console.log(currentConfiuration);

    setSchema(rosDiagnosticsConfiguration);
  }, []);

  // const configuration = useMemo(() => {
  //   Object.keys(topicStats).reduce((_, topic) => {
  //     return { ..._, topic: topicStats[topic] };
  //   }, {});
  //   return;
  // }, [topicStats]);

  const saveConfiguraton = async () => {
    console.log(config.current);
    // await KeyValue.set(
    //   "rosDiagnosticsConfiguration",
    //   JSON.stringify(config.current)
    // );
    // closeConfig();
    // showSnackBar();
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
          setConfguration((prev) => ({
            [""]: [{ name: "", type: "", hz: "", enable: true }],
            ...prev,
          }))
        }
        label="Topics"
      />
      {currentConfiuration === undefined &&
        Object.keys(configuration).map((_) => (
          <Section sectionName={_} topicList={configuration[_]} />
        ))}
      {/* <Section
        sectionName="Tranport"
        topicList={[{ name: "here", type: "doneone", hz: "1", enable: true }]}
      /> */}
      {/* <Box maxWidth={"50vw"} textAlign="left">
        <JsonSchemaForm
          jsonSchemaObject={
            jsonObjectFromCloud !== undefined ? jsonObjectFromCloud : schema
          }
          currentStateObject={config.current}
        />
      </Box> */}
      <Footer
        onCancel={closeConfig}
        onClick={saveConfiguraton}
        label={"Save"}
      />
    </Box>
  );
};
