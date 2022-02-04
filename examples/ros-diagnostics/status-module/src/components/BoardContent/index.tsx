import { Component, ReactNode } from "react";
import { TopicBoard } from "./TopicBoard";
import moduleConfig from "../../config/moduleConfig";
import { Header } from "./Header";
import { Topic } from "./Topic";

interface IBoardContentProps {
  onlineTopics: string[];
}

export class BoardContent extends Component<IBoardContentProps> {
  render(): ReactNode {
    return moduleConfig.sections.map((section) => {
      return (
        <TopicBoard>
          <Header title={section.title} />
          {section.contents.map((_, idx) => {
            return (
              <Topic
                topicState={
                  this.props.onlineTopics.indexOf(_.topic) >= 0
                    ? "good_standing"
                    : "not_found"
                }
                key={idx}
                name={_.topic}
              />
            );
          })}
        </TopicBoard>
      );
    });
  }
}
