import React, { FC, useEffect, useRef, useState, useMemo } from "react";
import RosTopicStats from "../../types/RosTopicStats";
import { Table } from "./Table";
import { TableHeader } from "./TableHeader";
import { TableBody } from "./TableBody";
import { TableRow } from "./TableRow";
import { TableSection } from "./TableSection";
import { TableDataCell } from "./TableDataCell";
import { Snackbar, Typography, Icon, Box } from "@formant/ui-sdk";
import { minHzForTopic, splitTopicStatsByConfig } from "./utils/index";
import { DialogComponent } from "../DialogComponent";
import { KeyValue, Authentication } from "@formant/data-sdk";
import { OnlineTopics } from "../../types/RosTopicStats";
interface ITableProps {
  tableHeaders: string[];
  topicStats: OnlineTopics;
  setOpenConfig: () => void;
  cloudConfig: any;
  showSnackBar: boolean;
  setShowSnackBar: () => void;
  onlineTopics: string[];
  currentConfiuration: OnlineTopics | undefined;
  openSnackBar: () => void;
}

export const TableComponent: FC<ITableProps> = ({
  tableHeaders,
  topicStats,
  setOpenConfig,
  cloudConfig,
  showSnackBar,
  setShowSnackBar,
  onlineTopics,
  currentConfiuration,
  openSnackBar,
}) => {
  // useEffect(() => {
  //   console.log(topicStats);
  // }, [topicStats]);
  const [openDialog, setOpenDialog] = useState(false);
  const [topicName, setTopicName] = useState("");
  const [msg, setmsg] = useState("Configuration saved");
  const deleteTopic = async () => {
    if (onlineTopics.includes(topicName)) {
      setmsg("Cannot delete online topic");
      handleCloseDialog();
      openSnackBar();
      return;
    }
    let temp = currentConfiuration;
    delete temp[topicName];
    if (await Authentication.waitTilAuthenticated()) {
      await KeyValue.set("rosDiagnosticsConfiguration", JSON.stringify(temp));
    }
    handleCloseDialog();
    openSnackBar();
  };

  const handleOpenDialog = (_: string) => {
    setTopicName(_);
    setOpenDialog(true);
  };

  const handleCloseDialog = () => {
    setOpenDialog(false);
  };

  // Probably merge online topics with configuration

  const topics = useMemo(() => {
    if (currentConfiuration === undefined) {
      return topicStats;
    }
    return currentConfiuration;
  }, [topicStats]);

  return (
    <>
      <Table columns={tableHeaders.length}>
        <TableHeader showConfig={setOpenConfig} headers={tableHeaders} />
        <TableBody>
          {Object.values(topics).map(
            (_: { section: string; contents: RosTopicStats }) => {
              return (
                <React.Fragment key={_.section}>
                  <TableRow>
                    {/* Fix Color in background 
                  Could add functionality to make collapse
                  */}
                    <TableDataCell content={_.section} />
                    <TableDataCell content={""} />
                    <TableDataCell content={""} />
                    <TableDataCell content={""} />
                  </TableRow>
                  {Object.values(_.contents).map(
                    (topic: {
                      topicName: string;
                      type: string;
                      hz: number;
                      enable?: boolean;
                    }) => {
                      return (
                        <TableRow key={topic.topicName}>
                          <TableDataCell
                            content={topic.topicName}
                            type={
                              onlineTopics.includes(topic.topicName)
                                ? minHzForTopic(
                                    topic.topicName,
                                    cloudConfig!
                                  ) <= topic.hz
                                  ? "good"
                                  : "bad"
                                : "unknown"
                            }
                          />
                          <TableDataCell content={topic.type} />
                          <TableDataCell content={Math.trunc(topic.hz)} />
                          <TableDataCell content={""} />
                          {/* <TableDataCell
                            center={true}
                            content={
                              <Box
                                onClick={() =>
                                  handleOpenDialog(topic.topicName)
                                }
                                sx={{
                                  height: 41,
                                  width: 41,
                                  display: "flex",
                                  alignItems: "center",
                                  justifyContent: "center",

                                  borderRadius: 5,
                                  ":hover": {
                                    backgroundColor: "#657197",
                                    cursor: "pointer",
                                  },
                                }}
                              >
                                <Icon name="delete" />
                              </Box>
                            }
                          /> */}
                        </TableRow>
                      );
                    }
                  )}
                </React.Fragment>
              );
            }
          )}
        </TableBody>
      </Table>
      <DialogComponent
        openDialog={openDialog}
        handleCloseDialog={handleCloseDialog}
        topicName={topicName}
        deleteTopic={deleteTopic}
      />
      <Snackbar
        open={showSnackBar}
        autoHideDuration={4000}
        onClose={setShowSnackBar}
        message={msg}
      />
    </>
  );
};
