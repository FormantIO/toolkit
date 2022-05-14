import * as React from "react";
import { useEffect } from "react";
import { Vector3 } from "three";
import produce from "immer";
import { useRecoilState } from "recoil";
import { Box, Button, Icon, Select, Stack, Typography } from "@formant/ui-sdk";
import styled from "styled-components";
import { defined, definedAndNotNull } from "../../../common/defined";
import { throttle } from "../../../common/throttle";
import { LayerType } from "../layers";
import { LayerRegistry } from "../layers/LayerRegistry";
import {
  LayerFields,
  LayerFieldValues,
  extractLayerFieldValues,
} from "../model/LayerField";
import {
  cloneSceneGraph,
  findSceneGraphElement,
  findSceneGraphParentElement,
  getSceneGraphElementParent,
  SceneGraphElement,
  visitSceneGraphElement,
  visitSceneGraphElementReverse,
} from "../model/SceneGraph";
import { TreeElement, TreePath, treePathEquals } from "../model/ITreeElement";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { UniverseSidebar } from "./sidebar";
import { UniverseViewer } from "./viewer";
import { AddLayerModal } from "./modals/AddLayerModal";
import { RenameLayerModal } from "./modals/RenameLayerModal";
import { SelectLocationModal } from "./modals/SelectLocationModal";
import { SelectTransformPathModal } from "./modals/SelectTransformPathModal";
import { FieldEditor } from "./FieldEditor";
import { UniverseSnackbar } from "./UniverseSnackbar";
import { sceneGraphAtom } from "../state/sceneGraph";

const Controls = styled.div`
  position: absolute;
  bottom: 1rem;
  left: 1rem;
`;

const Control = styled.div`
  > svg {
    width: 1rem;
    height: 1rem;
    color: white;
  }
  background: #bac4e2;
  opacity: 0.5;
  margin-bottom: 0.5rem;
  border-radius: 1rem;
  padding: 0;
  height: 2rem;
  width: 2rem;
  display: grid;
  place-items: center;
  cursor: pointer;
  transition: all 0.2s ease-in-out;

  &:hover {
    opacity: 1;
  }
`;

const UniverseContainer = styled.div`
  height: 100%;
`;

const PropertiesTitle = styled.div`
  border-bottom: #3b4668 solid 1px;
  margin-bottom: 0.5rem;
`;

const PropertyRow = styled.div`
  border-bottom: #3b4668 solid 1px;
  padding-bottom: 0.5rem;
  margin-bottom: 0.5rem;
`;

export interface IUniverseAppProps {
  universeData: IUniverseData;
  mode?: "edit" | "view" | "no-interaction";
  vr?: boolean;
  initialSceneGraph?: SceneGraphElement[];
  onSceneGraphChange?: (sceneGraph: SceneGraphElement[]) => void;
}

export interface IUniverseState {
  showingAddDialog: boolean;
  showingRenameDialog: boolean;
  showingTransformSelect: boolean;
  showingLocationStreamSelect: boolean;
  sidebarOpen: boolean;
  currentlySelectedElement: TreePath | undefined;
  currentContextName: string;
}

function stringToColor(str: string) {
  /* tslint:disable:no-bitwise */
  let hash = 0;
  for (let i = 0; i < str.length; i += 1) {
    // eslint-disable-next-line no-bitwise
    hash = str.charCodeAt(i) + ((hash << 5) - hash);
  }
  let color = "#";
  for (let i = 0; i < 3; i += 1) {
    // eslint-disable-next-line no-bitwise
    const value = (hash >> (i * 8)) & 0xff;
    color += `00${value.toString(16)}`.substr(-2);
  }
  /* tslint:enable:no-bitwise */
  return color;
}

export function UniverseApp(props: IUniverseAppProps) {
  const [showingAddDialog, setShowingAddDialog] = React.useState(false);
  const [showingRenameDialog, setShowingRenameDialog] = React.useState(false);
  const [showingTransformSelect, setShowingTransformSelect] =
    React.useState(false);
  const [showingLocationStreamSelect, setShowingLocationStreamSelect] =
    React.useState(false);
  const [sidebarOpen, setSidebarOpen] = React.useState(true);
  const [currentlySelectedElement, setCurrentlySelectedElement] =
    React.useState<TreePath | undefined>(undefined);
  const [currentContextName, setCurrentContextName] = React.useState("");
  const [currentlyEditingName, setCurrentlyEditingName] = React.useState("");
  const [viewer, setViewer] = React.useState<UniverseViewer | undefined>(
    undefined
  );
  const [sceneGraph, setSceneGraph] = useRecoilState(sceneGraphAtom);
  const [currentPath, setCurrentPath] = React.useState<TreePath>([]);
  const [currentlyEditingElement, setCurrentlyEditingElement] = React.useState<
    TreePath | undefined
  >(undefined);
  const [updatingSceneGraph, setUpdatingSceneGraph] = React.useState(false);
  const [fontLoaded, setFontLoaded] = React.useState(false);

  useEffect(() => {
    (async () => {
      const font = new FontFace(
        "Inter",
        "url(https://fonts.gstatic.com/s/inter/v11/UcC73FwrK3iLTeHuS_fvQtMwCp50KnMa1ZL7.woff2)"
      );
      await font.load();
      // @ts-ignore-next-line
      document.fonts.add(font);
      setFontLoaded(true);
    })();
  }, []);

  const persist = throttle(() => {
    if (updatingSceneGraph && props.onSceneGraphChange) {
      props.onSceneGraphChange(
        JSON.parse(JSON.stringify(sceneGraph)) as SceneGraphElement[]
      );
    }
  }, 1000);

  const hideAddDialog = () => {
    setShowingAddDialog(false);
  };

  const onAddLayer = (
    layerType: LayerType,
    dataSources?: UniverseDataSource[],
    fields?: LayerFieldValues,
    name?: string,
    deviceContext?: string
  ) => {
    hideAddDialog();
    if (viewer && currentPath) {
      const newElement = new SceneGraphElement(
        name || LayerRegistry.getCommonName(layerType),
        layerType,
        LayerRegistry.getDescription(layerType),
        dataSources
      );
      newElement.deviceContext = deviceContext;
      newElement.fieldValues = fields || {};
      let newPath: TreePath | undefined;
      const newSceneGraph = produce(sceneGraph, (draft) => {
        if (currentPath.length === 0) {
          draft.push(newElement);
          newPath = [draft.length - 1];
        } else {
          const el = definedAndNotNull(
            findSceneGraphElement(draft, currentPath)
          );
          el.children.push(newElement);
          newPath = [...currentPath, el.children.length - 1];
        }
      });
      setSceneGraph(newSceneGraph);
      viewer.addSceneGraphItem(newSceneGraph, defined(newPath), deviceContext);
    }
    persist();
  };

  const onViewerLoaded = async (el: UniverseViewer | null) => {
    if (viewer || !el) {
      return;
    }
    const v = el;
    setViewer(el || undefined);
    const newSceneGraph = produce(sceneGraph, (draft) => {
      const populateLayer = (
        e: SceneGraphElement,
        path: TreePath,
        i: number
      ) => {
        const pathThusFar = [...path, i];

        const newElement = new SceneGraphElement(
          e.name || LayerRegistry.getCommonName(e.type),
          e.type,
          LayerRegistry.getDescription(e.type),
          e.dataSources
        );
        newElement.deviceContext = e.deviceContext;
        newElement.fieldValues = e.fieldValues || {};
        let newPath;
        if (path.length === 0) {
          draft.push(newElement);
          newPath = [draft.length - 1];
        } else {
          const el2 = definedAndNotNull(findSceneGraphElement(draft, path));
          el2.children.push(newElement);
          newPath = [...path, el2.children.length - 1];
        }
        v.addSceneGraphItem(draft, newPath, e.deviceContext);

        if (e.children) {
          e.children.forEach((c, j) => populateLayer(c, pathThusFar, j));
        }

        if (e.position) {
          const element = findSceneGraphElement(draft, pathThusFar);
          definedAndNotNull(element).position = e.position;
          v.updatePositioning(draft, pathThusFar, e.position);
        }
      };
      if (props.initialSceneGraph) {
        props.initialSceneGraph.forEach((e, i) => {
          populateLayer(e, [], i);
        });
      }
    });
    setSceneGraph(newSceneGraph);
    setUpdatingSceneGraph(true);
  };

  const showAddDialog = (cp: TreePath) => {
    setShowingAddDialog(true);
    setCurrentPath(cp);
  };

  const showRenameDialog = (cp: TreePath) => {
    if (cp.length > 0) {
      setShowingRenameDialog(true);
      setCurrentPath(cp);
      setCurrentlyEditingName(
        definedAndNotNull(findSceneGraphElement(sceneGraph, cp)).name
      );
    }
  };

  const hideRenameDialog = () => {
    setShowingRenameDialog(false);
  };

  const onLayerAdded = (
    layerType: LayerType,
    dataSources?: UniverseDataSource[],
    fields?: LayerFields,
    name?: string,
    deviceContext?: string
  ) => {
    onAddLayer(
      layerType,
      dataSources,
      extractLayerFieldValues(fields || {}),
      name,
      deviceContext
    );
  };

  const onRemoveItem = (path: TreePath) => {
    if (!viewer) {
      return;
    }

    const e = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    visitSceneGraphElementReverse(
      e,
      (_, epath) => {
        if (
          currentlyEditingElement &&
          treePathEquals(currentlyEditingElement, epath)
        ) {
          defined(viewer).toggleEditing(
            sceneGraph,
            currentlyEditingElement,
            false
          );
          setCurrentlyEditingElement(undefined);
        }
        defined(viewer).removeSceneGraphItem(sceneGraph, epath);
      },
      path
    );
    const newSceneGraph = produce(sceneGraph, (draft) => {
      const sgParent = getSceneGraphElementParent(draft, path);
      if (sgParent) {
        sgParent.children.splice(path[path.length - 1], 1);
      } else {
        draft.splice(path[path.length - 1], 1);
      }
    });
    setSceneGraph(newSceneGraph);
    persist();
  };

  const onDuplicateItem = (path: TreePath) => {
    if (!viewer) {
      return;
    }

    const newSceneGraph = produce(sceneGraph, (draft) => {
      const sgParent = getSceneGraphElementParent(draft, path);
      if (sgParent) {
        const newEl = cloneSceneGraph(
          defined(sgParent.children[path[path.length - 1]])
        );
        sgParent.children.push(newEl);
        path.pop();
        const newPath = [...path, sgParent.children.length - 1];
        visitSceneGraphElement(
          newEl,
          (_, epath) => {
            defined(viewer).addSceneGraphItem(draft, epath);
          },
          newPath
        );
      } else {
        const newEl = cloneSceneGraph(defined(draft[path[path.length - 1]]));
        draft.push(newEl);
        path.pop();
        const newPath = [...path, draft.length - 1];
        visitSceneGraphElement(
          newEl,
          (_, epath) => {
            defined(viewer).addSceneGraphItem(draft, epath);
          },
          newPath
        );
      }
    });
    setSceneGraph(newSceneGraph);
    persist();
  };

  const onRenameLayer = (name: string) => {
    hideRenameDialog();
    if (currentPath) {
      const newSceneGraph = produce(sceneGraph, (draft) => {
        const el = definedAndNotNull(findSceneGraphElement(draft, currentPath));
        el.name = name;
      });
      setSceneGraph(newSceneGraph);
    }
    persist();
  };

  const onIconInteracted = (path: TreePath, icon: number) => {
    if (path.length === 0 || !viewer) {
      return;
    }
    const newSceneGraph = produce(sceneGraph, (draft) => {
      const el = definedAndNotNull(findSceneGraphElement(draft, path));
      if (icon === 0) {
        el.visible = !el.visible;

        visitSceneGraphElement(
          el,
          (e, epath) => {
            if (el.visible && !e.visible) {
              // stop early if we start encountering elements that should stay invisible
              return false;
            }
            if (
              currentlyEditingElement &&
              treePathEquals(currentlyEditingElement, epath)
            ) {
              defined(viewer).toggleEditing(
                draft,
                currentlyEditingElement,
                false
              );
              e.editing = false;
              setCurrentlyEditingElement(undefined);
            }
            defined(viewer).toggleVisible(draft, epath, el.visible);
            return false;
          },
          path
        );
      } else if (icon === 1) {
        const isEditing = el.editing;
        if (currentlyEditingElement) {
          const editingElement = definedAndNotNull(
            findSceneGraphElement(draft, currentlyEditingElement)
          );
          editingElement.editing = false;
          viewer.toggleEditing(draft, currentlyEditingElement, false);
          setCurrentlyEditingElement(undefined);
        }
        if (!isEditing) {
          el.editing = true;
          setCurrentlyEditingElement(path);
          viewer.toggleEditing(draft, path, el.editing);
        }
      }
    });
    setSceneGraph(newSceneGraph);

    persist();
  };

  const onSceneGraphElementEdited = (path: TreePath, transform: Vector3) => {
    if (viewer) {
      const newSceneGraph = produce(sceneGraph, (draft) => {
        const el = definedAndNotNull(findSceneGraphElement(draft, path));
        if (el.position.type === "manual") {
          el.position = {
            type: "manual",
            x: transform.x,
            y: transform.y,
            z: transform.z,
          };
        }
      });
      setSceneGraph(newSceneGraph);
    }
    persist();
  };

  const recenter = () => {
    if (viewer) {
      viewer.recenter();
    }
  };

  const onItemSelected = (path?: TreePath) => {
    setCurrentlySelectedElement(path);
    const newSceneGraph = produce(sceneGraph, (draft) => {
      let element: SceneGraphElement | null | undefined;
      let parentContext: string | undefined;
      let currentContext: string | undefined;
      if (path) {
        element = findSceneGraphElement(draft, path);
        const parent = findSceneGraphParentElement(
          draft,
          path,
          (el) => el.deviceContext !== undefined
        );
        if (parent) {
          parentContext = parent.deviceContext;
        }
        if (element) currentContext = element.deviceContext || parentContext;
      }

      if (currentContext)
        props.universeData.getDeviceContextName(currentContext).then((_) => {
          if (_) {
            setCurrentContextName(_);
          }
        });
    });
    setSceneGraph(newSceneGraph);
  };

  const onChangePositionType = (positionType: string) => {
    const newSceneGraph = produce(sceneGraph, (draft) => {
      const element = definedAndNotNull(
        findSceneGraphElement(draft, defined(currentlySelectedElement))
      );
      if (positionType === "manual") {
        element.position = {
          type: "manual",
          x: 0,
          y: 0,
          z: 0,
        };
      } else if (positionType === "gps") {
        element.position = {
          type: "gps",
          relativeToLongitude: 0,
          relativeToLatitude: 0,
        };
      } else if (positionType === "transform tree") {
        element.position = {
          type: "transform tree",
        };
      }
    });
    setSceneGraph(newSceneGraph);
    persist();
  };

  const onSelectTransformPath = (name: string, end: string) => {
    setShowingTransformSelect(false);
    const newSceneGraph = produce(sceneGraph, (draft) => {
      const element = definedAndNotNull(
        findSceneGraphElement(draft, defined(currentlySelectedElement))
      );
      element.position = {
        type: "transform tree",
        stream: name,
        end,
      };
      defined(viewer).updatePositioning(
        draft,
        defined(currentlySelectedElement),
        element.position
      );
    });
    setSceneGraph(newSceneGraph);
    persist();
  };

  const onSelectLocationStream = (
    name: string,
    relativeToLong: number,
    relativeToLat: number
  ) => {
    setShowingLocationStreamSelect(false);
    const newSceneGraph = produce(sceneGraph, (draft) => {
      const element = definedAndNotNull(
        findSceneGraphElement(draft, defined(currentlySelectedElement))
      );
      element.position = {
        type: "gps",
        stream: name,
        relativeToLongitude: relativeToLong,
        relativeToLatitude: relativeToLat,
      };
      defined(viewer).updatePositioning(
        draft,
        defined(currentlySelectedElement),
        element.position
      );
    });
    setSceneGraph(newSceneGraph);
    persist();
  };

  const showTransformSelect = () => {
    setShowingTransformSelect(true);
  };

  const hideTransformSelect = () => {
    setShowingTransformSelect(false);
  };

  const showLocationStreamSelect = () => {
    setShowingLocationStreamSelect(true);
  };

  const hideLocationStreamSelect = () => {
    setShowingLocationStreamSelect(false);
  };

  const toggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };

  const onFieldChanged = (fieldId: string, value: string) => {
    if (viewer) {
      viewer.notifyFieldChanged(
        sceneGraph,
        defined(currentlySelectedElement),
        fieldId,
        value
      );
    }
  };

  const buildSubTree = (
    element: SceneGraphElement,
    path: TreePath,
    inheritedColor?: string
  ): TreeElement => {
    const color = element.deviceContext
      ? stringToColor(element.deviceContext)
      : inheritedColor;
    return {
      title: element.name,
      textColor: color,
      icons: [
        {
          icon: element.visible ? "eye" : "eye_closed",
          description: "click to toggle visibility",
        },
        {
          icon: "edit",
          description: "edit",
          color: element.editing ? "#18d2ff" : "white",
        },
        {
          icon: "help",
          description: LayerRegistry.getDescription(element.type),
        },
      ],
      children: element.children.map((_, i) =>
        buildSubTree(_, [...path, i], color)
      ),
    };
  };

  const buildTree = (): TreeElement[] => [
    {
      title: "Universe",
      icons: [
        {
          icon: "help",
          description: "This is your entire collection of layers.",
        },
      ],
      children: sceneGraph.map((_, i) => buildSubTree(_, [i])),
    },
  ];

  const { mode, vr } = props;
  let element: SceneGraphElement | null | undefined;
  let hasParentContext = false;
  let parentContext: string | undefined;
  let currentContext: string | undefined;
  if (currentlySelectedElement) {
    element = findSceneGraphElement(sceneGraph, currentlySelectedElement);
    const parent = findSceneGraphParentElement(
      sceneGraph,
      currentlySelectedElement,
      (el) => el.deviceContext !== undefined
    );
    if (parent) {
      parentContext = parent.deviceContext;
      hasParentContext = true;
    }
    if (element) currentContext = element.deviceContext || parentContext;
  }

  const showSidebar = mode === "edit" && sidebarOpen;

  let fields: LayerFields = {};
  const fieldValues: { [key in string]: string | undefined } = {};
  if (element) {
    fields = LayerRegistry.getFields(element.type, "edit");
    Array.from(Object.keys(fields)).forEach((key) => {
      fieldValues[key] = element?.fieldValues[key].value;
    });
  }

  return (
    <>
      <UniverseSnackbar />
      <UniverseContainer>
        <Box
          display={showSidebar ? "grid" : "block"}
          gridTemplateColumns={showSidebar ? "370px 1fr" : undefined}
          sx={{ height: "100%" }}
        >
          {showSidebar && (
            <UniverseSidebar
              onAdd={showAddDialog}
              onRemove={onRemoveItem}
              onDuplicate={onDuplicateItem}
              onRename={showRenameDialog}
              tree={buildTree()}
              onIconInteracted={onIconInteracted}
              onItemSelected={onItemSelected}
            >
              {element !== undefined && element !== null && (
                <>
                  <PropertiesTitle>Properties</PropertiesTitle>
                  <div>
                    {currentContext !== undefined && (
                      <PropertyRow>
                        <>
                          device:{" "}
                          {currentContext !== undefined
                            ? currentContextName
                            : "none"}
                        </>
                      </PropertyRow>
                    )}
                    <div>
                      <Stack spacing={4}>
                        <div>
                          <div>positioning</div>
                          <Select
                            label="Positioning"
                            value={element.position.type}
                            onChange={onChangePositionType}
                            items={[
                              "manual",
                              ...(element.deviceContext || hasParentContext
                                ? ["transform tree", "gps"]
                                : []),
                            ].map((_) => ({ label: _, value: _ }))}
                          />
                        </div>
                        {element.position.type === "manual" && (
                          <div>
                            <Typography variant="body1">
                              x: {element.position.x.toFixed(4)}
                              <br />
                              y: {element.position.y.toFixed(4)}
                              <br />
                              z: {element.position.z.toFixed(4)}
                            </Typography>
                          </div>
                        )}
                        {element.position.type === "gps" && (
                          <div>
                            <Typography variant="body1">
                              stream: {element.position.stream}
                              <br />
                              relative to longitude:{" "}
                              {element.position.relativeToLongitude}
                              <br />
                              relative to latitude:{" "}
                              {element.position.relativeToLatitude}
                            </Typography>
                            <Button
                              variant="contained"
                              onClick={showLocationStreamSelect}
                            >
                              Select
                            </Button>
                          </div>
                        )}
                        {element.position.type === "transform tree" && (
                          <div>
                            <Typography variant="body1">
                              stream: {element.position.stream}
                              <br />
                              transform: {element.position.end}
                            </Typography>
                            <Button
                              variant="contained"
                              onClick={showTransformSelect}
                            >
                              Select
                            </Button>
                          </div>
                        )}
                        {Object.entries(fields).map(([fieldId, field]) => (
                          <FieldEditor
                            key={fieldId}
                            fieldId={fieldId}
                            field={field}
                            value={fieldValues[fieldId]}
                            onChange={onFieldChanged}
                          />
                        ))}
                      </Stack>
                    </div>
                  </div>
                </>
              )}
            </UniverseSidebar>
          )}
          <Box
            sx={{ position: "relative", overflow: "hidden", height: "100%" }}
          >
            {fontLoaded && (
              <UniverseViewer
                ref={onViewerLoaded}
                onSceneGraphElementEdited={onSceneGraphElementEdited}
                universeData={props.universeData}
                vr={vr}
              />
            )}
            <Controls>
              <Control onClick={recenter}>
                <Icon name="recenter" />
              </Control>
              {mode === "edit" && (
                <Control onClick={toggleSidebar}>
                  <Icon name="edit" />
                </Control>
              )}
            </Controls>
          </Box>
        </Box>

        {showingAddDialog && (
          <AddLayerModal
            onCancel={hideAddDialog}
            onAddLayer={onLayerAdded}
            universeData={props.universeData}
            deviceContext={element?.deviceContext || parentContext}
          />
        )}
        {showingRenameDialog && (
          <RenameLayerModal
            name={currentlyEditingName}
            onCancel={hideRenameDialog}
            onRenameLayer={onRenameLayer}
          />
        )}
        {showingTransformSelect && currentContext && (
          <SelectTransformPathModal
            deviceContext={currentContext}
            universeData={props.universeData}
            onCancel={hideTransformSelect}
            onSelect={onSelectTransformPath}
          />
        )}
        {showingLocationStreamSelect && currentContext && (
          <SelectLocationModal
            deviceContext={currentContext}
            universeData={props.universeData}
            onCancel={hideLocationStreamSelect}
            onSelect={onSelectLocationStream}
          />
        )}
      </UniverseContainer>
    </>
  );
}
