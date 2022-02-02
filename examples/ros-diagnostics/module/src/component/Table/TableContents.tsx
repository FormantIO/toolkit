import { Component } from "react";
import RosTopicStats from "../../types/RosTopicStats";
import moduleConfig from "../../config/moduleConfig";
import { Table } from "semantic-ui-react";
import "./Table.scss";
interface ITableContentsProps {
  topicStats: RosTopicStats[];
}

class TableContents extends Component<ITableContentsProps> {
  public constructor(props: any) {
    super(props);
  }
  public render() {
    const topicsSplit = splitTopicStatsByConfig(this.props.topicStats);
    return (
      <Table className="formant-table" celled>
        <Table.Header>
          <Table.Row>
            <Table.HeaderCell>Section</Table.HeaderCell>
            <Table.HeaderCell>Name</Table.HeaderCell>
            <Table.HeaderCell>Type</Table.HeaderCell>
            <Table.HeaderCell>Hz</Table.HeaderCell>
            <Table.HeaderCell>Health</Table.HeaderCell>
          </Table.Row>
        </Table.Header>
        <Table.Body>
          {topicsSplit.map((_) => {
            return _.contents.map((content, index) => {
              return (
                <Table.Row>
                  {index === 0 && (
                    <Table.Cell rowSpan={_.contents.length}>
                      {_.title}
                    </Table.Cell>
                  )}
                  <Table.Cell>{content.name}</Table.Cell>
                  <Table.Cell>{content.type}</Table.Cell>
                  <Table.Cell>{Math.trunc(content.hz)}</Table.Cell>
                  <Table.Cell>
                    {minHzForTopic(content.name) <= content.hz
                      ? "True"
                      : "False"}
                  </Table.Cell>
                </Table.Row>
              );
            });
          })}
        </Table.Body>
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
export default TableContents;
