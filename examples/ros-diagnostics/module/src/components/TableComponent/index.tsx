import React, { FC, useMemo } from "react";
import RosTopicStats from "../../types/RosTopicStats";
import { Table } from "./Table";
import { TableHeader } from "./TableHeader";
import { TableBody } from "./TableBody";
import { TableRow } from "./TableRow";
import { TableDataCell } from "./TableDataCell";
import { OnlineTopics } from "../../types/RosTopicStats";
interface ITableProps {
  tableHeaders: string[];
  topicStats: OnlineTopics;
  setOpenConfig: () => void;
  onlineTopics: string[];
  currentConfiuration: OnlineTopics | undefined;
  openSnackBar: () => void;
}

export const TableComponent: FC<ITableProps> = ({
  tableHeaders,
  topicStats,
  setOpenConfig,
  onlineTopics,
  currentConfiuration,
}) => {
  // TODO: merge new topics to configuration

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
                                ? "good"
                                : "bad"
                            }
                          />
                          <TableDataCell content={topic.type} />
                          <TableDataCell content={Math.trunc(topic.hz)} />
                          <TableDataCell content={""} />
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
    </>
  );
};
