import { Box, Button, Icon } from "@formant/ui-sdk";
import {
  FC,
  useRef,
  useState,
  useLayoutEffect,
  useCallback,
  useEffect,
} from "react";
import { JsonSchemaForm } from "./JsonSchemaForm/index";
import { KeyValue } from "@formant/data-sdk";

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
}) => {
  const [schema, setSchema] = useState<any>();
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
          minHz: {
            type: "integer",
            title: "minHz",
          },
        },
      };
    });

    setSchema(rosDiagnosticsConfiguration);
  }, [topicStats]);

  const saveConfiguraton = async () => {
    await KeyValue.set(
      "rosDiagnosticsConfiguration",
      JSON.stringify(config.current)
    );
    closeConfig();
    showSnackBar();
  };

  return (
    <Box
      position={"relative"}
      display={"flex"}
      flexDirection="column"
      alignItems="center"
      paddingTop={"70px"}
      paddingBottom={"100px"}
    >
      <Box
        position={"fixed"}
        top={0}
        left={0}
        height={"70px"}
        display={"flex"}
        alignItems="center"
        justifyContent={"center"}
        marginLeft={2}
        borderRadius={25}
      >
        <Box
          height={"40px"}
          width="40px"
          borderRadius={25}
          display={"flex"}
          alignItems="center"
          justifyContent={"center"}
          onClick={closeConfig}
          sx={{
            ":hover": {
              backgroundColor: "#3b4668",
              cursor: "pointer",
            },
          }}
        >
          <Icon name="arrow-left" />
        </Box>
      </Box>
      <Box maxWidth={"50vw"} textAlign="left">
        <JsonSchemaForm
          jsonSchemaObject={
            jsonObjectFromCloud !== undefined ? jsonObjectFromCloud : schema
          }
          currentStateObject={config.current}
        />
      </Box>
      <Box
        height={"80px"}
        padding={"20px"}
        boxShadow={"0 0 1.25rem #1c1e2d"}
        position={"fixed"}
        bottom={0}
        width={"100vw"}
        zIndex={20}
        sx={{
          backgroundColor: "#2d3855",
        }}
        textAlign="right"
      >
        <Button
          onClick={saveConfiguraton}
          variant="contained"
          size="large"
          color="secondary"
        >
          Update
        </Button>
      </Box>
    </Box>
  );
};
