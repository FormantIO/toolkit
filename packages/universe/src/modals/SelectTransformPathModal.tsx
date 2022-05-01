import * as React from "react";
import { Component } from "react";
import { DialogContentText, Select } from "@formant/ui-sdk";
import { ITransformNode } from "../../../data-sdk/src/model/ITransformNode";
import { defined } from "../../../common/defined";
import { TreeElement, TreePath } from "../ITreeElement";
import { IUniverseData } from "../IUniverseData";
import { SortableTree } from "../SortableTree";
import { Modal } from "./Modal";

interface ISelectTransformPathModalProps {
  deviceContext: string;
  universeData: IUniverseData;
  onSelect: (tree: string, end: string) => void;
  onCancel: () => void;
}

interface ISelectTransformPathModalState {
  selected: TreePath | undefined;
  items: Map<string, TreeElement>;
  mapName: string | undefined;
}

export class SelectTransformPathModal extends Component<
  ISelectTransformPathModalProps,
  ISelectTransformPathModalState
> {
  async componentDidMount() {
    const transformTrees =
      await this.props.universeData.getLatestTransformTrees(
        this.props.deviceContext
      );
    const trees = new Map<string, TreeElement>();
    transformTrees.forEach((tree) =>
      trees.set(
        tree.streamName,
        this.buildTransformTreeElement(tree.transformTree)
      )
    );
    this.setState({
      items: trees,
      mapName: Array.from(trees.keys())[0],
    });
  }

  getSelectedTitle(treeElements: TreeElement[], path: TreePath): string {
    const i = path.shift();
    if (i === undefined) {
      throw new Error("Invalid path");
    }
    const element = defined(treeElements[i]);
    if (path.length === 0) {
      return element.title;
    }
    return this.getSelectedTitle(element.children || [], path);
  }

  onSelectTransform = () => {
    if (this.state.mapName) {
      this.props.onSelect(
        this.state.mapName,
        this.getSelectedTitle(
          [defined(this.state.items.get(this.state.mapName))],
          defined(this.state.selected)
        )
      );
    }
  };

  onSelectTransformItem = (item: TreePath) => {
    this.setState({
      selected: item,
    });
  };

  onChangeTree = (target: string) => {
    this.setState({
      mapName: target,
      selected: undefined,
    });
  };

  buildTransformTreeElement(tree: ITransformNode): TreeElement {
    return {
      title: defined(tree.name),
      children: tree.children?.map((_) => this.buildTransformTreeElement(_)),
    };
  }

  public render() {
    const { onCancel } = this.props;
    let items;
    if (this.state?.mapName) {
      items = this.state.items.get(this.state.mapName);
    }
    const treeNames = Array.from(this.state?.items.keys() || []);

    return (
      <Modal
        open
        title="Select Transform Path"
        acceptText="Select"
        onAccept={this.onSelectTransform}
        acceptDisabled={!this.state?.selected || !this.state?.mapName}
        onClose={onCancel}
      >
        <DialogContentText>
          Select the transform path you would like to use
        </DialogContentText>
        {items && (
          <>
            <Select
              label="TF"
              value={this.state?.mapName}
              onChange={this.onChangeTree}
              items={treeNames.map((_) => ({ label: _, value: _ }))}
            />
            <SortableTree
              items={[items]}
              onSelected={this.onSelectTransformItem}
              selected={this.state.selected}
            />
          </>
        )}
      </Modal>
    );
  }
}
