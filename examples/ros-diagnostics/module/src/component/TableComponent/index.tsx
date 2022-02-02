import { Component, ReactNode } from "react";
import styles from "./index.module.scss";
import RosTopicStats from "../../types/RosTopicStats";
import moduleConfig from "../../config/moduleConfig";

interface ITableProps {
  tableHeaders: string[];
  topicStats: RosTopicStats[];
}

enum healthState {
  unknown,
  low,
  good,
}

export class TableComponent extends Component<ITableProps> {
  public constructor(props: any) {
    super(props);
  }
  render(): ReactNode {
    const topicsSplit = splitTopicStatsByConfig(this.props.topicStats);

    return (
      <table
        className={styles.table}
        style={{
          gridTemplateColumns: `repeat(${this.props.tableHeaders.length}, 1fr)`,
        }}
      >
        <tr>
          {/* Header */}
          {this.props.tableHeaders.map((_) => {
            return <th className={styles["table-header"]}>{_}</th>;
          })}
        </tr>
        <tbody>
          {topicsSplit.map((_) => {
            return _.contents.map((content, index) => {
              return (
                <tr className={styles["table-row"]}>
                  {index === 0 && (
                    <td
                      className={styles["table-section"]}
                      rowSpan={_.contents.length}
                    >
                      {_.title}
                    </td>
                  )}
                  <td
                    className={`${styles["table-data-cell"]} 
                    ${styles["table-data-cell-name"]} 
                    ${
                      styles[
                        _.title === "other"
                          ? "table-data-cell-unknown"
                          : minHzForTopic(content.name) <= content.hz
                          ? "table-data-cell-good"
                          : "table-data-cell-bad"
                      ]
                    }`}
                  >
                    {content.name}
                  </td>
                  <td className={styles["table-data-cell"]}>{content.type}</td>
                  <td className={styles["table-data-cell"]}>
                    {Math.trunc(content.hz)}
                  </td>
                  {/* <td className={styles["table-data-cell"]}>
                    {minHzForTopic(content.name) <= content.hz
                      ? "True"
                      : "False"}
                  </td> */}
                </tr>
              );
            });
          })}
        </tbody>
      </table>
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
