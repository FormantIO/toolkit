import * as React from "react";
import { Component } from "react";
import { defined } from "../../../common/defined";
import { ITransformNode } from "../../../data-sdk/src/model/ITransformNode";
import { TreeElement, TreePath } from "../ITreeElement";
import { IUniverseData } from "../IUniverseData";
import { Typography, Button } from "@formant/ui-sdk";
import { SortableTree } from "../SortableTree";
import { Modal } from "../modals/Modal";

interface ISelectTransformPathModalProps {
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
  buildTransformTreeElement(tree: ITransformNode): TreeElement {
    return {
      title: defined(tree.name),
      children: tree.children?.map((_) => this.buildTransformTreeElement(_)),
    };
  }

  async componentDidMount() {
    const transformTrees = await this.props.universeData.getTransformTrees();
    const trees = new Map<string, TreeElement>();
    transformTrees.forEach((tree) =>
      trees.set(tree.name, this.buildTransformTreeElement(tree.transformTree))
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

  onChangeTree = (ev: React.ChangeEvent<HTMLSelectElement>) => {
    this.setState({
      mapName: ev.target.value,
      selected: undefined,
    });
  };

  public render() {
    const { onCancel } = this.props;
    let items;
    if (this.state.mapName) {
      items = this.state.items.get(this.state.mapName);
    }
    const treeNames = Array.from(this.state.items.keys());

    return (
      <Modal>
        <Typography variant="h1">Select Transform Path</Typography>
        Select the transform path you'd like to use
        <hr />
        {items && (
          <>
            <select value={this.state.mapName} onChange={this.onChangeTree}>
              {treeNames.map((_) => (
                <option key={_} value={_}>
                  {_}
                </option>
              ))}
            </select>
            <SortableTree
              items={[items]}
              onSelected={this.onSelectTransformItem}
              selected={this.state.selected}
            />
          </>
        )}
        <div>
          <Button onClick={onCancel}>Cancel</Button>
          <Button
            onClick={this.onSelectTransform}
            disabled={!this.state.selected || !this.state.mapName}
          >
            Select
          </Button>
        </div>
      </Modal>
    );
  }
}
