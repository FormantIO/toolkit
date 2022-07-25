import { JsonSchemaForm } from "./JsonSchemaForm";
import { Footer } from "./Footer";
import { Box } from "@formant/ui-sdk";
import { Header } from "./Header";
import { FC, useEffect, useMemo, useRef, useState } from "react";

interface IAddTopicProps {
  onBack: () => void;
  onSave: (_: any) => void;
  currentConfiguration: any;
}

export const AddTopic: FC<IAddTopicProps> = ({
  onBack,
  onSave,
  currentConfiguration,
}) => {
  const newTopicForm = {
    type: "object",
    title: "New Topic form",
    properties: {
      name: {
        title: "name",
        type: "string",
      },
      type: {
        title: "type",
        type: "string",
      },
      section: {
        title: "section",
        type: "string",
      },
      minHz: {
        title: "minHz",
        type: "integer",
      },
    },
  };

  let newTopicState = useRef({ name: "", type: "", section: "", minHz: 0 });

  const addNewTopic = () => {
    if (newTopicState.current.name.length === 0) return;

    currentConfiguration[newTopicState.current.name] = {
      section: newTopicState.current.section,
      type: newTopicState.current.type,
      minHz: newTopicState.current.minHz,
    };
    onSave(currentConfiguration);
  };

  return (
    <Box
      position={"relative"}
      display={"flex"}
      flexDirection="column"
      alignItems="center"
      paddingTop={10}
    >
      <Header label="Add topic" onBack={onBack} />
      <Box maxWidth={"50vw"} textAlign="left">
        <JsonSchemaForm
          currentStateObject={newTopicState.current}
          jsonSchemaObject={newTopicForm}
        />
      </Box>
      <Footer
        disabled={newTopicState.current.name === ""}
        onClick={addNewTopic}
        label="Save"
      />
    </Box>
  );
};
