import { FC, useEffect, useRef, useState } from "react";
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
interface ITableProps {
  tableHeaders: string[];
  topicStats: RosTopicStats[];
  setOpenConfig: () => void;
  cloudConfig: any;
  showSnackBar: boolean;
  setShowSnackBar: () => void;
  onlineTopics: string[];
  currentConfiuration: any;
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

  useEffect(() => {
    console.log(topicStats)
  }, [topicStats])
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

  if (topicStats === undefined || topicStats.length === 0) return <></>;

  return (
    <>
      <Table columns={tableHeaders.length}>
        <TableHeader showConfig={setOpenConfig} headers={tableHeaders} />
        <TableBody>
          {splitTopicStatsByConfig(topicStats, cloudConfig!).map((_) => {
            return _.contents.map((content, index) => {
              return (
                <TableRow key={content.name}>
                  {/* {index === 0 && (
                    <TableSection
                      title={_.title === "undefined" ? "Other" : _.title}
                      rowSpan={_.contents.length}
                    />
                  )} */}
                  <TableDataCell
                    content={content.name}
                    type={
                      onlineTopics!.includes(content.name)
                        ? minHzForTopic(content.name, cloudConfig!) <=
                          content.hz
                          ? "good"
                          : "bad"
                        : "unknown"
                    }
                  />
                  <TableDataCell content={content.type} />
                  <TableDataCell content={Math.trunc(content.hz)} />
                  <TableDataCell
                    center={true}
                    content={
                      <Box
                        onClick={() => handleOpenDialog(content.name)}
                        sx={{
                          height: 41,
                          width: 41,
                          display: "flex",
                          alignItems: "center",
                          justifyContent: "center",
                          backgroundColor: "#282f45",
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
                  />
                </TableRow>
              );
            });
          })}
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
