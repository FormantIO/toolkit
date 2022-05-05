import * as React from "react";
import { useEffect } from "react";
import { DialogContentText, Select } from "@formant/ui-sdk";
import { ITransformNode } from "../../../../data-sdk/src/model/ITransformNode";
import { defined } from "../../../../common/defined";
import { TreeElement, TreePath } from "../../model/ITreeElement";
import { IUniverseData } from "../../model/IUniverseData";
import { SortableTree } from "../SortableTree";
import { Modal } from "./Modal";
import { fork } from "../../../../common/fork";

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

export function SelectTransformPathModal(
  props: ISelectTransformPathModalProps
) {
  const [selected, setSelected] = React.useState<TreePath | undefined>(
    undefined
  );
  const [items, setItems] = React.useState<Map<string, TreeElement>>(new Map());
  const [mapName, setMapName] = React.useState<string | undefined>(undefined);

  const buildTransformTreeElement = (tree: ITransformNode): TreeElement => ({
    title: defined(tree.name),
    children: tree.children?.map((_) => buildTransformTreeElement(_)),
  });

  useEffect((): void => {
    fork(
      (async () => {
        const transformTrees = await props.universeData.getLatestTransformTrees(
          props.deviceContext
        );
        const trees = new Map<string, TreeElement>();
        transformTrees.forEach((tree) =>
          trees.set(
            tree.streamName,
            buildTransformTreeElement(tree.transformTree)
          )
        );
        setItems(trees);
        setMapName(Array.from(trees.keys())[0]);
      })()
    );
  }, []);

  const getSelectedTitle = (
    treeElements: TreeElement[],
    path: TreePath
  ): string => {
    const i = path.shift();
    if (i === undefined) {
      throw new Error("Invalid path");
    }
    const element = defined(treeElements[i]);
    if (path.length === 0) {
      return element.title;
    }
    return getSelectedTitle(element.children || [], path);
  };

  const onSelectTransform = () => {
    if (mapName) {
      props.onSelect(
        mapName,
        getSelectedTitle([defined(items.get(mapName))], defined(selected))
      );
    }
  };

  const onSelectTransformItem = (item: TreePath) => {
    setSelected(item);
  };

  const onChangeTree = (target: string) => {
    setMapName(target);
    setSelected(undefined);
  };

  const { onCancel } = props;
  let treeItems;
  if (mapName) {
    treeItems = items.get(mapName);
  }
  const treeNames = Array.from(items.keys() || []);

  return (
    <Modal
      open
      title="Select Transform Path"
      acceptText="Select"
      onAccept={onSelectTransform}
      acceptDisabled={!selected || !mapName}
      onClose={onCancel}
    >
      <DialogContentText>
        Select the transform path you would like to use
      </DialogContentText>
      {treeItems && (
        <>
          <Select
            label="TF"
            value={mapName}
            onChange={onChangeTree}
            items={treeNames.map((_) => ({ label: _, value: _ }))}
          />
          <SortableTree
            items={[treeItems]}
            onSelected={onSelectTransformItem}
            selected={selected}
          />
        </>
      )}
    </Modal>
  );
}
