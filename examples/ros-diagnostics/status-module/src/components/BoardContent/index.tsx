import { Component, ReactNode } from "react";
import { TopicBoard } from "./TopicBoard";
import nodes from "../../config/moduleConfig";
import { Header } from "./Header";
import { Topic } from "./Topic";
import { rosNodes } from "../../config/types/node";

interface IBoardContentProps {
  onlineNodes: rosNodes;
}

export class BoardContent extends Component<IBoardContentProps> {
  checkNodeState = (node: string): boolean => {
    if (this.props.onlineNodes === undefined) return false;
    console.log(this.props.onlineNodes);

    let idx = this.props.onlineNodes.keys.indexOf(node);
    return idx >= 0 ? this.props.onlineNodes.values[idx] : false;
  };

  render(): ReactNode {
    console.log(this.props.onlineNodes);
    return (
      <TopicBoard>
        <Header title={"Nodes"} />
        {nodes.map((_, idx) => {
          return (
            <Topic
              key={idx}
              name={_}
              topicState={
                this.checkNodeState(_) ? "good_standing" : "not_found"
              }
            />
          );
        })}

        {/* {section.contents.map((_, idx) => {
            return (
              <Topic
                topicState={
                  this.props.onlineNodes.indexOf(_.topic) >= 0
                    ? "good_standing"
                    : "not_found"
                }
                key={idx}
                name={_.topic}
              />
            );
          })} */}
      </TopicBoard>
    );
  }
}
