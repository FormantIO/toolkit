import * as React from "react";
import { Component } from "react";
import { Vector3 } from "three";
import { defined, definedAndNotNull } from "../../common/defined";
import { LayerType } from "./layers";
import { LayerRegistry } from "./layers/LayerRegistry";
import {
  extractLayerFieldValues,
  LayerFields,
} from "./layers/UniverseLayerContent";
import {
  cloneSceneGraph,
  findSceneGraphElement,
  findSceneGraphParentElement,
  getSceneGraphElementParent,
  SceneGraphElement,
  visitSceneGraphElement,
  visitSceneGraphElementReverse,
} from "./SceneGraph";
import { TreeElement, TreePath, treePathEquals } from "./ITreeElement";
import { IUniverseData, UniverseDataSource } from "./IUniverseData";
import { UniverseSidebar } from "./sidebar";
import { UniverseViewer } from "./viewer";
import "./layers";
import { AddLayerModal } from "./modals/AddLayerModal";
import { RenameLayerModal } from "./modals/RenameLayerModal";
import { SelectLocationModal } from "./modals/SelectLocationModal";
import { SelectTransformPathModal } from "./modals/SelectTransformPathModal";
import { Button, Icon, Select, Stack, Typography } from "@formant/ui-sdk";
import styled from "styled-components";
import { Mosaic, MosaicWindow } from "react-mosaic-component";
import classNames from "classnames";
// @ts-ignore-next-line
import styleInject from "style-inject";
// @ts-ignore-next-line
import css from "./universe.css";
styleInject(css);

const MosaicContainer = styled.div`
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

export interface IUniverseProps {
  universeData: IUniverseData;
}

export interface IUniverseState {
  showingAddDialog: boolean;
  showingRenameDialog: boolean;
  showingTransformSelect: boolean;
  showingLocationStreamSelect: boolean;
  sidebarOpen: boolean;
  currentlySelectedElement: TreePath | undefined;
}

export type ViewId = "properties" | "viewer";

const TITLE_MAP: Record<ViewId, string> = {
  properties: "Properties",
  viewer: "Viewer",
};

export class Universe extends Component<IUniverseProps, IUniverseState> {
  constructor(props: IUniverseProps) {
    super(props);
    this.state = {
      showingAddDialog: false,
      showingRenameDialog: false,
      showingTransformSelect: false,
      showingLocationStreamSelect: false,
      sidebarOpen: true,
      currentlySelectedElement: undefined,
    };
  }
  private currentlyEditingName: string = "";

  private viewer: UniverseViewer | undefined;

  private sceneGraph: SceneGraphElement[] = [];

  private currentPath: TreePath = [];

  private currentlyEditingElement: TreePath | undefined;

  onViewerLoaded = (el: UniverseViewer | null) => {
    this.viewer = el || undefined;
    const deviceId = this.props.universeData.getDeviceContexts()[0].deviceId;
    const deviceName = this.props.universeData.getDeviceContextName(deviceId);
    // Add some nice default layers
    this.onAddLayer("data", undefined, undefined, deviceName, deviceId);
    this.onAddLayer("ground");
    this.currentPath = [0];
    this.onAddLayer(
      "label",
      undefined,
      {
        label_text: {
          type: "text",
          value: deviceName,
          description: "",
          placeholder: "",
          name: "",
        },
      },
      "Label",
      deviceId
    );
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

  private onAddLayer = (
    layerType: LayerType,
    dataSources?: UniverseDataSource[],
    fields?: LayerFields,
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
      newElement.fieldValues = extractLayerFieldValues(fields || {});
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
      this.viewer.addSceneGraphItem(newPath);
      this.forceUpdate();
    }
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
  };

  stringToColor(str: string) {
    /* tslint:disable:no-bitwise */
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
      hash = str.charCodeAt(i) + ((hash << 5) - hash);
    }
    let color = "#";
    for (let i = 0; i < 3; i++) {
      const value = (hash >> (i * 8)) & 0xff;
      color += ("00" + value.toString(16)).substr(-2);
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
          return;
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
    this.setState({
      sidebarOpen: !this.state.sidebarOpen,
    });
  };

  public render() {
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
        (el) => {
          return el.deviceContext !== undefined;
        }
      );
      if (parent) {
        parentContext = parent.deviceContext;
        hasParentContext = true;
      }
      if (element) currentContext = element.deviceContext || parentContext;
    }

    return (
      <MosaicContainer>
        <Mosaic<ViewId>
          className={classNames("mosaic-blueprint-theme")}
          renderTile={(id, path) => (
            // @ts-ignore-next-line
            <MosaicWindow<ViewId>
              path={path}
              toolbarControls={<div></div>}
              title={TITLE_MAP[id]}
            >
              {id === "viewer" && (
                <>
                  <UniverseViewer
                    ref={this.onViewerLoaded}
                    sceneGraph={this.sceneGraph}
                    onSceneGraphElementEdited={this.onSceneGraphElementEdited}
                    universeData={this.props.universeData}
                    deviceId={this.props.universeData.deviceId}
                  />
                  <Controls>
                    <Control onClick={this.recenter}>
                      <Icon name="recenter" />
                    </Control>
                    <Control onClick={this.toggleSidebar}>
                      <Icon name="edit" />
                    </Control>
                  </Controls>
                </>
              )}
              {id === "properties" && (
                <UniverseSidebar
                  onAdd={this.showAddDialog}
                  onRemove={this.onRemoveItem}
                  onDuplicate={this.onDuplicateItem}
                  onRename={this.showRenameDialog}
                  tree={this.buildTree()}
                  onIconInteracted={this.onIconInteracted}
                  onItemSelected={this.onItemSelected}
                >
                  {(element === undefined || element === null) && <></>}
                  {element !== undefined && element !== null && (
                    <>
                      <PropertiesTitle>Properties</PropertiesTitle>
                      <div>
                        {currentContext !== undefined && (
                          <PropertyRow>
                            device:{" "}
                            {currentContext !== undefined
                              ? this.props.universeData.getDeviceContextName(
                                  currentContext
                                )
                              : "none"}
                          </PropertyRow>
                        )}
                        <div>
                          <Stack spacing={2}>
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
                          </Stack>
                        </div>
                      </div>
                    </>
                  )}
                </UniverseSidebar>
              )}
            </MosaicWindow>
          )}
          initialValue={{
            direction: "row",
            first: "properties",
            second: "viewer",
            splitPercentage: 20,
          }}
        />

        {this.state.showingAddDialog && (
          <AddLayerModal
            onCancel={this.hideAddDialog}
            onAddLayer={this.onAddLayer}
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
        {this.state.showingTransformSelect && (
          <SelectTransformPathModal
            universeData={this.props.universeData}
            onCancel={this.hideTransformSelect}
            onSelect={this.onSelectTransformPath}
          />
        )}
        {this.state.showingLocationStreamSelect && (
          <SelectLocationModal
            universeData={this.props.universeData}
            onCancel={this.hideLocationStreamSelect}
            onSelect={this.onSelectLocationStream}
          />
        )}
      </MosaicContainer>
    );
  }
}

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
