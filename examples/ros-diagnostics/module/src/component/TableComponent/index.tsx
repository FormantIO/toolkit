import { Component, ReactNode } from "react";
import styles from "./index.module.scss";
import RosTopicStats from "../../types/RosTopicStats";
import moduleConfig from "../../config/moduleConfig";
import { Table } from "./Table";
import { TableHeader } from "./TableHeader";
import { TableBody } from "./TableBody";
import { TableRow } from "./TableRow";
import { TableSection } from "./TableSection";
import { TableDataCell } from "./TableDataCell";

interface ITableProps {
  tableHeaders: string[];
  topicStats: RosTopicStats[];
}
export class TableComponent extends Component<ITableProps> {
  public constructor(props: any) {
    super(props);
  }
  render(): ReactNode {
    const topicsSplit = splitTopicStatsByConfig(this.props.topicStats);

    return (
      <Table columns={this.props.tableHeaders.length}>
        <TableHeader headers={this.props.tableHeaders} />
        <TableBody>
          {topicsSplit.map((_) => {
            return _.contents.map((content, index) => {
              return (
                <TableRow>
                  {index === 0 && (
                    <TableSection title={_.title} rowSpan={_.contents.length} />
                  )}
                  <TableDataCell
                    content={content.name}
                    type={
                      _.title === "other"
                        ? "unknown"
                        : minHzForTopic(content.name) <= content.hz
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
    );
  }
}

function minHzForTopic(topicName: string) {
  return moduleConfig.sections.reduce(
    (acc, section) =>
      Math.max(
        acc,
        section.contents.reduce(
          (a, _) =>
            _.topic === topicName ? Math.max(a, _.minHz) : Math.max(a, 0),
          0
        )
      ),
    0
  );
}

function splitTopicStatsByConfig(
  topicStats: RosTopicStats[]
): { title: string; contents: RosTopicStats[] }[] {
  const remainingTopics = [...topicStats];

  const topicsSplitBySection = moduleConfig.sections.map((section) => {
    return {
      title: section.title,
      contents: section.contents.map((content) => {
        const foundIndex = remainingTopics.findIndex(
          (_) => _.name === content.topic
        );
        const foundTopic =
          foundIndex >= 0
            ? remainingTopics.splice(foundIndex, 1)[0]
            : { name: content.topic, type: "unknown", hz: 0 };
        return foundTopic;
      }),
    };
  });
  topicsSplitBySection.push({ title: "other", contents: remainingTopics });
  return topicsSplitBySection;
}
