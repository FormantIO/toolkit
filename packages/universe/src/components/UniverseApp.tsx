import * as React from "react";
import { Component } from "react";
import { Vector3 } from "three";
import { RecoilRoot } from "recoil";
import RecoilNexus from "recoil-nexus";
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

export class UniverseApp extends Component<IUniverseAppProps, IUniverseState> {
  private currentlyEditingName: string = "";

  private viewer: UniverseViewer | undefined;

  private sceneGraph: SceneGraphElement[] = [];

  private currentPath: TreePath = [];

  private currentlyEditingElement: TreePath | undefined;

  private updatingSceneGraph: boolean = false;

  private persist = throttle(() => {
    if (this.updatingSceneGraph && this.props.onSceneGraphChange) {
      this.props.onSceneGraphChange(
        JSON.parse(JSON.stringify(this.sceneGraph)) as SceneGraphElement[]
      );
    }
  }, 1000);

  constructor(props: IUniverseAppProps) {
    super(props);
    this.state = {
      showingAddDialog: false,
      showingRenameDialog: false,
      showingTransformSelect: false,
      showingLocationStreamSelect: false,
      sidebarOpen: true,
      currentlySelectedElement: undefined,
      currentContextName: "",
    };
  }

  onViewerLoaded = async (el: UniverseViewer | null) => {
    this.viewer = el || undefined;
    const populateLayer = (e: SceneGraphElement, path: TreePath, i: number) => {
      this.currentPath = path;
      const pathThusFar = [...path, i];
      this.onAddLayer(
        e.type,
        e.dataSources,
        e.fieldValues,
        e.name,
        e.deviceContext
      );
      if (e.children) {
        e.children.forEach((c, j) => populateLayer(c, pathThusFar, j));
      }

      if (e.position) {
        const element = findSceneGraphElement(this.sceneGraph, pathThusFar);
        definedAndNotNull(element).position = e.position;
        defined(this.viewer).updatePositioning(pathThusFar, e.position);
      }
    };
    if (this.props.initialSceneGraph) {
      this.props.initialSceneGraph.forEach((e, i) => {
        populateLayer(e, [], i);
      });
    }
    this.updatingSceneGraph = true;
    this.forceUpdate();
  };

  private showAddDialog = (currentPath: TreePath) => {
    this.setState({
      showingAddDialog: true,
    });
    this.currentPath = currentPath;
  };

  private showRenameDialog = (currentPath: TreePath) => {
    if (currentPath.length > 0) {
      this.setState({
        showingRenameDialog: true,
      });
      this.currentPath = currentPath;
      this.currentlyEditingName = definedAndNotNull(
        findSceneGraphElement(this.sceneGraph, currentPath)
      ).name;
    }
  };

  private hideAddDialog = () => {
    this.setState({
      showingAddDialog: false,
    });
  };

  private hideRenameDialog = () => {
    this.setState({
      showingRenameDialog: false,
    });
  };

  private onLayerAdded = (
    layerType: LayerType,
    dataSources?: UniverseDataSource[],
    fields?: LayerFields,
    name?: string,
    deviceContext?: string
  ) => {
    this.onAddLayer(
      layerType,
      dataSources,
      extractLayerFieldValues(fields || {}),
      name,
      deviceContext
    );
  };

  private onAddLayer = (
    layerType: LayerType,
    dataSources?: UniverseDataSource[],
    fields?: LayerFieldValues,
    name?: string,
    deviceContext?: string
  ) => {
    this.hideAddDialog();
    if (this.viewer && this.currentPath) {
      const newElement = new SceneGraphElement(
        name || LayerRegistry.getCommonName(layerType),
        layerType,
        LayerRegistry.getDescription(layerType),
        dataSources
      );
      newElement.deviceContext = deviceContext;
      newElement.fieldValues = fields || {};
      let newPath;
      if (this.currentPath.length === 0) {
        this.sceneGraph.push(newElement);
        newPath = [this.sceneGraph.length - 1];
      } else {
        const el = definedAndNotNull(
          findSceneGraphElement(this.sceneGraph, this.currentPath)
        );
        el.children.push(newElement);
        newPath = [...this.currentPath, el.children.length - 1];
      }
      this.viewer.addSceneGraphItem(newPath, deviceContext);
      this.forceUpdate();
    }
    this.persist();
  };

  private onRemoveItem = (path: TreePath) => {
    if (!this.viewer) {
      return;
    }

    const e = definedAndNotNull(findSceneGraphElement(this.sceneGraph, path));
    visitSceneGraphElementReverse(
      e,
      (_, epath) => {
        if (
          this.currentlyEditingElement &&
          treePathEquals(this.currentlyEditingElement, epath)
        ) {
          defined(this.viewer).toggleEditing(
            this.currentlyEditingElement,
            false
          );
          this.currentlyEditingElement = undefined;
        }
        defined(this.viewer).removeSceneGraphItem(epath);
      },
      path
    );
    const sgParent = getSceneGraphElementParent(this.sceneGraph, path);
    if (sgParent) {
      sgParent.children.splice(path[path.length - 1], 1);
    } else {
      this.sceneGraph.splice(path[path.length - 1], 1);
    }
    this.forceUpdate();
    this.persist();
  };

  private onDuplicateItem = (path: TreePath) => {
    if (!this.viewer) {
      return;
    }
    const sgParent = getSceneGraphElementParent(this.sceneGraph, path);
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
          defined(this.viewer).addSceneGraphItem(epath);
        },
        newPath
      );
    } else {
      const newEl = cloneSceneGraph(
        defined(this.sceneGraph[path[path.length - 1]])
      );
      this.sceneGraph.push(newEl);
      path.pop();
      const newPath = [...path, this.sceneGraph.length - 1];
      visitSceneGraphElement(
        newEl,
        (_, epath) => {
          defined(this.viewer).addSceneGraphItem(epath);
        },
        newPath
      );
    }
    this.forceUpdate();
    this.persist();
  };

  private onRenameLayer = (name: string) => {
    this.hideRenameDialog();
    if (this.currentPath) {
      const el = definedAndNotNull(
        findSceneGraphElement(this.sceneGraph, this.currentPath)
      );
      el.name = name;
      this.forceUpdate();
    }
    this.persist();
  };

  private onIconInteracted = (path: TreePath, icon: number) => {
    if (path.length === 0 || !this.viewer) {
      return;
    }
    const el = definedAndNotNull(findSceneGraphElement(this.sceneGraph, path));
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
            this.currentlyEditingElement &&
            treePathEquals(this.currentlyEditingElement, epath)
          ) {
            defined(this.viewer).toggleEditing(
              this.currentlyEditingElement,
              false
            );
            e.editing = false;
            this.currentlyEditingElement = undefined;
          }
          defined(this.viewer).toggleVisible(epath, el.visible);
          return false;
        },
        path
      );
    } else if (icon === 1) {
      const isEditing = el.editing;
      if (this.currentlyEditingElement) {
        const editingElement = definedAndNotNull(
          findSceneGraphElement(this.sceneGraph, this.currentlyEditingElement)
        );
        editingElement.editing = false;
        this.viewer.toggleEditing(this.currentlyEditingElement, false);
        this.currentlyEditingElement = undefined;
      }
      if (!isEditing) {
        el.editing = true;
        this.currentlyEditingElement = path;
        this.viewer.toggleEditing(path, el.editing);
      }
    }
    this.forceUpdate();
    this.persist();
  };

  private onSceneGraphElementEdited = (path: TreePath, transform: Vector3) => {
    if (this.viewer) {
      const el = definedAndNotNull(
        findSceneGraphElement(this.sceneGraph, path)
      );
      if (el.position.type === "manual") {
        el.position = {
          type: "manual",
          x: transform.x,
          y: transform.y,
          z: transform.z,
        };
        this.forceUpdate();
      }
    }
    this.persist();
  };

  private recenter = () => {
    if (this.viewer) {
      this.viewer.recenter();
    }
  };

  private onItemSelected = (path?: TreePath) => {
    this.setState({
      currentlySelectedElement: path,
    });
    let element: SceneGraphElement | null | undefined;
    let parentContext: string | undefined;
    let currentContext: string | undefined;
    if (path) {
      element = findSceneGraphElement(this.sceneGraph, path);
      const parent = findSceneGraphParentElement(
        this.sceneGraph,
        path,
        (el) => el.deviceContext !== undefined
      );
      if (parent) {
        parentContext = parent.deviceContext;
      }
      if (element) currentContext = element.deviceContext || parentContext;
    }

    if (currentContext)
      this.props.universeData.getDeviceContextName(currentContext).then((_) => {
        if (_) {
          this.setState({
            currentContextName: _,
          });
        }
      });
  };

  private onChangePositionType = (positionType: string) => {
    const element = definedAndNotNull(
      findSceneGraphElement(
        this.sceneGraph,
        defined(this.state.currentlySelectedElement)
      )
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
    this.forceUpdate();
    this.persist();
  };

  private onSelectTransformPath = (name: string, end: string) => {
    this.setState({
      showingTransformSelect: false,
    });
    const element = definedAndNotNull(
      findSceneGraphElement(
        this.sceneGraph,
        defined(this.state.currentlySelectedElement)
      )
    );
    element.position = {
      type: "transform tree",
      stream: name,
      end,
    };
    defined(this.viewer).updatePositioning(
      defined(this.state.currentlySelectedElement),
      element.position
    );

    this.forceUpdate();
    this.persist();
  };

  private onSelectLocationStream = (
    name: string,
    relativeToLong: number,
    relativeToLat: number
  ) => {
    this.setState({
      showingLocationStreamSelect: false,
    });
    const element = definedAndNotNull(
      findSceneGraphElement(
        this.sceneGraph,
        defined(this.state.currentlySelectedElement)
      )
    );
    element.position = {
      type: "gps",
      stream: name,
      relativeToLongitude: relativeToLong,
      relativeToLatitude: relativeToLat,
    };
    defined(this.viewer).updatePositioning(
      defined(this.state.currentlySelectedElement),
      element.position
    );
    this.forceUpdate();
    this.persist();
  };

  private showTransformSelect = () => {
    this.setState({
      showingTransformSelect: true,
    });
  };

  private hideTransformSelect = () => {
    this.setState({
      showingTransformSelect: false,
    });
  };

  private showLocationStreamSelect = () => {
    this.setState({
      showingLocationStreamSelect: true,
    });
  };

  private hideLocationStreamSelect = () => {
    this.setState({
      showingLocationStreamSelect: false,
    });
  };

  private toggleSidebar = () => {
    this.setState((s) => ({
      sidebarOpen: !s.sidebarOpen,
    }));
  };

  private onFieldChanged = (fieldId: string, value: string) => {
    if (this.viewer) {
      this.viewer.notifyFieldChanged(
        defined(this.state.currentlySelectedElement),
        fieldId,
        value
      );
    }
  };

  stringToColor(str: string) {
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

  private buildSubTree(
    element: SceneGraphElement,
    path: TreePath,
    inheritedColor?: string
  ): TreeElement {
    const color = element.deviceContext
      ? this.stringToColor(element.deviceContext)
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
        this.buildSubTree(_, [...path, i], color)
      ),
    };
  }

  private buildTree(): TreeElement[] {
    return [
      {
        title: "Universe",
        icons: [
          {
            icon: "help",
            description: "This is your entire collection of layers.",
          },
        ],
        children: this.sceneGraph.map((_, i) => this.buildSubTree(_, [i])),
      },
    ];
  }

  render() {
    const { mode, vr } = this.props;
    const { sidebarOpen } = this.state;
    let element: SceneGraphElement | null | undefined;
    let hasParentContext = false;
    let parentContext: string | undefined;
    let currentContext: string | undefined;
    if (this.state.currentlySelectedElement) {
      element = findSceneGraphElement(
        this.sceneGraph,
        this.state.currentlySelectedElement
      );
      const parent = findSceneGraphParentElement(
        this.sceneGraph,
        this.state.currentlySelectedElement,
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
                onAdd={this.showAddDialog}
                onRemove={this.onRemoveItem}
                onDuplicate={this.onDuplicateItem}
                onRename={this.showRenameDialog}
                tree={this.buildTree()}
                onIconInteracted={this.onIconInteracted}
                onItemSelected={this.onItemSelected}
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
                              ? this.state.currentContextName
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
                              onChange={this.onChangePositionType}
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
                                onClick={this.showLocationStreamSelect}
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
                                onClick={this.showTransformSelect}
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
                              initialValue={fieldValues[fieldId]}
                              onChange={this.onFieldChanged}
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
              <UniverseViewer
                ref={this.onViewerLoaded}
                sceneGraph={this.sceneGraph}
                onSceneGraphElementEdited={this.onSceneGraphElementEdited}
                universeData={this.props.universeData}
                vr={vr}
              />
              <Controls>
                <Control onClick={this.recenter}>
                  <Icon name="recenter" />
                </Control>
                {mode === "edit" && (
                  <Control onClick={this.toggleSidebar}>
                    <Icon name="edit" />
                  </Control>
                )}
              </Controls>
            </Box>
          </Box>

          {this.state.showingAddDialog && (
            <AddLayerModal
              onCancel={this.hideAddDialog}
              onAddLayer={this.onLayerAdded}
              universeData={this.props.universeData}
              deviceContext={element?.deviceContext || parentContext}
            />
          )}
          {this.state.showingRenameDialog && (
            <RenameLayerModal
              name={this.currentlyEditingName}
              onCancel={this.hideRenameDialog}
              onRenameLayer={this.onRenameLayer}
            />
          )}
          {this.state.showingTransformSelect && currentContext && (
            <SelectTransformPathModal
              deviceContext={currentContext}
              universeData={this.props.universeData}
              onCancel={this.hideTransformSelect}
              onSelect={this.onSelectTransformPath}
            />
          )}
          {this.state.showingLocationStreamSelect && currentContext && (
            <SelectLocationModal
              deviceContext={currentContext}
              universeData={this.props.universeData}
              onCancel={this.hideLocationStreamSelect}
              onSelect={this.onSelectLocationStream}
            />
          )}
        </UniverseContainer>
      </>
    );
  }
}