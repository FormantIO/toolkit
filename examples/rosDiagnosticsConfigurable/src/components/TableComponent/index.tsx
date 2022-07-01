import { FC, useEffect, useRef, useState } from "react";
import RosTopicStats from "../../types/RosTopicStats";
import { Table } from "./Table";
import { TableHeader } from "./TableHeader";
import { TableBody } from "./TableBody";
import { TableRow } from "./TableRow";
import { TableSection } from "./TableSection";
import { TableDataCell } from "./TableDataCell";
import { ModuleConfig } from "../ModuleConfig";
import { Snackbar } from "@formant/ui-sdk";
import { KeyValue, Authentication } from "@formant/data-sdk";
import {
  minHzForTopic,
  createJsonSchemaObjectFromConfig,
  splitTopicStatsByConfig,
  splitTopicsForSecction,
} from "./utils/index";

interface ITableProps {
  tableHeaders: string[];
  topicStats: RosTopicStats[];
}

type Topic = {
  topic: string;
  minHz: string;
};

export const TableComponent: FC<ITableProps> = ({
  tableHeaders,
  topicStats,
}) => {
  const [cloudConfig, setCloudConfig] = useState<{
    [key: string]: {
      title: string;
      contents: Topic[];
    };
  }>();

  const [openConfig, setOpenConfig] = useState(false);
  const [showSnackBar, setShowSnackBar] = useState(false);
  const [jsonObjectFromCloud, setJsonObjectFromCloud] = useState<any>();
  const [currentConfig, setCurrenConfig] = useState();

  const getCurrentConfig = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      let config = await KeyValue.get("rosDiagnosticsConfiguration");
      setCurrenConfig(JSON.parse(config));
      setJsonObjectFromCloud(
        createJsonSchemaObjectFromConfig(JSON.parse(config))
      );
      setCloudConfig(splitTopicsForSecction(JSON.parse(config)));
    }
  };

  useEffect(() => {
    getCurrentConfig();
  }, [openConfig]);

  if (topicStats === undefined || topicStats.length === 0) return <></>;
  return openConfig ? (
    <ModuleConfig
      closeConfig={() => setOpenConfig(false)}
      topicStats={topicStats}
      showSnackBar={() => setShowSnackBar(true)}
      jsonObjectFromCloud={jsonObjectFromCloud}
      currentConfiuration={currentConfig}
    />
  ) : (
    <>
      <Table columns={tableHeaders.length}>
        <TableHeader
          showConfig={() => setOpenConfig(true)}
          headers={tableHeaders}
        />
        <TableBody>
          {splitTopicStatsByConfig(topicStats, cloudConfig!).map((_) => {
            return _.contents.map((content, index) => {
              return (
                <TableRow key={content.name}>
                  {index === 0 && (
                    <TableSection
                      title={_.title === "undefined" ? "Other" : _.title}
                      rowSpan={_.contents.length}
                    />
                  )}
                  <TableDataCell
                    content={content.name}
                    type={
                      _.title === "other"
                        ? "unknown"
                        : minHzForTopic(content.name, cloudConfig!) <=
                          content.hz
                        ? "good"
                        : "bad"
                    }
                  />
                  <TableDataCell content={content.type} />
                  <TableDataCell content={Math.trunc(content.hz)} />
                </TableRow>
              );
            });
          })}
        </TableBody>
      </Table>
      <Snackbar
        open={showSnackBar}
        autoHideDuration={4000}
        onClose={() => setShowSnackBar(false)}
        message={"Configuration saved"}
      />
    </>
  );
};
