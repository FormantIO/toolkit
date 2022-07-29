import { Box, Button, Icon } from "@formant/ui-sdk";
import { FC, useState, useLayoutEffect } from "react";
import { JsonSchemaForm } from "./JsonSchemaForm/index";
import { KeyValue } from "@formant/data-sdk";
import { Footer } from "./Footer";
import { Header } from "./Header";
import { AddTopic } from "./AddTopic";
import { Section } from "./Section/index";
interface IModuleConfig {
  topicStats: any;
  closeConfig: () => void;
  showSnackBar: () => void;
  jsonObjectFromCloud: any;
  currentConfiuration?: any;
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
  const config = { current: {} };

  const rosDiagnosticsConfiguration = {
    type: "object",
    title: "Ros Diagnostics Configuration",
    properties: {},
  };

  useLayoutEffect(() => {
    if (jsonObjectFromCloud) return;
    topicStats.forEach((_: any) => {
      (rosDiagnosticsConfiguration as any).properties[_.name] = {
        type: "object",
        title: _.name,
        properties: {
          section: {
            type: "string",
            title: "section",
          },
          type: {
            type: "string",
            title: "type",
            default: _.type,
          },
          minHz: {
            type: "integer",
            title: "minHz",
            default: 0,
          },
        },
      };
    });

    setSchema(rosDiagnosticsConfiguration);
  }, [topicStats]);

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
        onClick={() => setShowAddTopic(true)}
        label="Topics"
      />
      <Section />
      <Box maxWidth={"50vw"} textAlign="left">
        <JsonSchemaForm
          jsonSchemaObject={
            jsonObjectFromCloud !== undefined ? jsonObjectFromCloud : schema
          }
          currentStateObject={config.current}
        />
      </Box>
      <Footer
        onCancel={closeConfig}
        onClick={saveConfiguraton}
        label={"Save"}
      />
    </Box>
  );
};
