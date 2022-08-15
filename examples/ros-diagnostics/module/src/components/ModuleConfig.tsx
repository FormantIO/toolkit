import { Box, Button, Icon } from "@formant/ui-sdk";
import React, {
  FC,
  useState,
  useLayoutEffect,
  useMemo,
  useCallback,
} from "react";
import { JsonSchemaForm } from "./JsonSchemaForm/index";
import { KeyValue } from "@formant/data-sdk";
import { Footer } from "./Footer";
import { Header } from "./Header";
import { AddTopic } from "./AddTopic";
import { Section } from "./Section/index";
import RosTopicStats, { OnlineTopics } from "../types/RosTopicStats";
import { v4 as uuidv4 } from "uuid";
import { Options } from "./Section/Options";
import styles from "./options.module.scss";
import "../index.css";
import _, { get, unset } from "lodash";
import { updatePath } from "./updatePath";
import { DialogComponent } from "./DialogComponent";

interface IModuleConfig {
  topicStats: OnlineTopics;
  closeConfig: () => void;
  showSnackBar: () => void;
  jsonObjectFromCloud: any;
  currentConfiuration?: any;
}

export const ModuleConfig: FC<IModuleConfig> = ({
  topicStats,
  closeConfig,
  showSnackBar,
  currentConfiuration,
}) => {
  const [index, setIndex] = useState("");
  const [openDialog, setOpenDialog] = useState(false);
  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [path, setPath] = useState<string[]>([]);
  const [showAddTopic, setShowAddTopic] = useState(false);
  const [availableSections, setAvailableSections] = useState<
    { name: string; path: string }[]
  >([]);
  const [configuration, setConfguration] = useState<OnlineTopics>({
    [uuidv4()]: {
      section: "default",
      contents: { default: { topicName: "", type: "", hz: 0, enabled: true } },
    },
  });

  window.addEventListener("click", (e) => {
    e.stopPropagation();
    const options = document.getElementById("options");
    if (options) {
      options.classList.add("fade-out");
      options.classList.remove("fade-in");
      options.style.display = "none";
    }
  });

  const handleOpenOptions = (
    e: React.MouseEvent<HTMLDivElement, globalThis.MouseEvent>,
    section: string,
    path: string[]
  ) => {
    e.stopPropagation();
    setPath(path);
    const _sections = Object.keys(configuration).filter(
      (_) => configuration[_].section !== section
    );

    setAvailableSections(
      _sections.map((_) => ({ name: configuration[_].section, path: _ }))
    );

    const { innerHeight, innerWidth } = window;
    const { clientY, clientX, pageY, pageX } = e; //Mouse coordinates

    const options = document.getElementById("options")!;
    options.style.display = "flex";
    options.classList.remove("fade-out");
    options.classList.add("fade-in");
    let yOffset = 40;
    let xOffset = options.clientWidth - 20;
    //Handle if click close to the bottom
    if (innerHeight - 80 < clientY + options.clientHeight) {
      yOffset = options.clientHeight - 20;
    }

    options.style.top = pageY - yOffset + "px";
    options.style.left = pageX - xOffset + "px";
  };

  useLayoutEffect(() => {
    if (currentConfiuration === undefined) {
      setConfguration(topicStats);
      return;
    }
    setConfguration(currentConfiuration);
  }, []);

  const saveConfiguraton = async () => {
    // console.log(configuration);
    // return;
    const err = Object.keys(configuration).filter(
      (_) => configuration[_].section.length < 1
    );
    if (err.length > 0) return; //Section name can't be empty ;
    await KeyValue.set(
      "rosDiagnosticsConfiguration",
      JSON.stringify(configuration)
    );
    closeConfig();
    showSnackBar();
  };

  const handleMoveToSection = (
    e: React.MouseEvent<HTMLDivElement, MouseEvent>,
    sectionPath: string
  ) => {
    e.stopPropagation();
    const options = document.getElementById("options")!;
    const selectedTopic = get(configuration, path);

    const deepConfigurationCopy = JSON.parse(JSON.stringify(configuration));
    //creates a deep copy of the current configuration, to remove selected topic
    //and updates the configuration adding the topic to desire section with a new id
    unset(deepConfigurationCopy, path);
    setConfguration(
      updatePath(
        deepConfigurationCopy,
        `[${sectionPath}][contents][${uuidv4()}]`,
        selectedTopic
      )
    );

    options.classList.add("fade-out");
    options.classList.remove("fade-in");
    options.style.display = "none";
  };

  const handleCloseDialog = useCallback(() => {
    setOpenDialog(false);
  }, []);

  const handleOpenDialog = useCallback(() => {
    setOpenDialog(true);
  }, []);

  const handleDeleteTopic = useCallback(() => {
    setConfguration((prev) => updatePath(prev, path + "[enabled]", false));
    handleCloseDialog();
  }, [path]);

  return showAddTopic ? (
    <AddTopic
      currentConfiguration={currentConfiuration}
      onBack={() => setShowAddTopic(false)}
      onSave={async (_) => {
        await KeyValue.set("rosDiagnosticsConfiguration", JSON.stringify(_));
        closeConfig();
        showSnackBar();
      }}
    />
  ) : (
    <Box
      position={"relative"}
      display={"flex"}
      flexDirection="column"
      alignItems="center"
      paddingBottom={"100px"}
    >
      <Header
        onBack={closeConfig}
        buttonLabel="ADD TOPIC"
        onClick={() =>
          //ADD empty section to current configuration
          setConfguration((prev) => ({
            [uuidv4()]: {
              section: "",
              contents: {
                [uuidv4()]: { topicName: "", type: "", hz: 0, enable: true },
              },
            },
            ...prev,
          }))
        }
        label="Topics"
      />
      {
        <div id="options" className={styles.options}>
          <Options
            handleMoveToSection={handleMoveToSection}
            sections={availableSections}
            handleDeleteTopic={handleOpenDialog}
          />
        </div>
      }
      {Object.keys(configuration).map((_) => {
        return (
          <Section
            key={_}
            index={_}
            params={configuration}
            setParams={setConfguration}
            topicList={configuration[_].contents}
            handleOpenOptions={handleOpenOptions}
          />
        );
      })}

      <DialogComponent
        openDialog={openDialog}
        title={"DELETE TOPIC"}
        handleCloseDialog={handleCloseDialog}
        description={`Delete ${
          get(configuration, path + "[topicName]") ?? ""
        } ?`}
        onOk={handleDeleteTopic}
      />

      <Footer
        onCancel={closeConfig}
        onClick={saveConfiguraton}
        label={"Save"}
      />
    </Box>
  );
};
