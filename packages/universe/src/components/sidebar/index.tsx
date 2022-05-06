import { Button } from "@formant/ui-sdk";
import React from "react";
import styled from "styled-components";
import {
  TreeElement,
  TreePath,
  treePathEquals,
} from "../../model/ITreeElement";
import { SortableTree } from "../SortableTree";

const SidebarContainer = styled.div`
  background-color: #2d3855;
  padding: 1rem;
  display: grid;
  grid-template-rows: 25rem auto 3rem;
  gap: 1rem;
  height: 100%;
`;

const PropertiesSectionDiv = styled.div`
  display: grid;
  margin-bottom: 2rem;
`;

const ButtonsDiv = styled.div`
  display: flex;
  flex-direction: row;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
`;

const TreeArea = styled.div`
  overflow-y: scroll;
  padding-right: 1rem;
`;

export interface IUniverseSidebarProps {
  onAdd: (currentPath: TreePath) => void;
  onRemove: (currentPath: TreePath) => void;
  onDuplicate: (currentPath: TreePath) => void;
  onRename: (currentPath: TreePath) => void;
  onIconInteracted?: (currentPath: TreePath, iconIndex: number) => void;
  onItemSelected: (currentPath?: TreePath) => void;
  tree: TreeElement[];
  children?: React.ReactNode;
}

export function UniverseSidebar({
  onRename,
  onAdd,
  onRemove,
  onDuplicate,
  onIconInteracted,
  onItemSelected,
  tree,
  children,
}: IUniverseSidebarProps) {
  const [selected, setSelected] = React.useState<TreePath | undefined>(
    undefined
  );

  const onTreeNodeSelect = (path: TreePath) => {
    const currentSelected = selected && treePathEquals(selected, path);
    setSelected(currentSelected ? undefined : path);
    onItemSelected(currentSelected ? undefined : path.slice(1));
  };

  const onTreeNodeIconSelect = (path: TreePath, iconIndex: number) => {
    if (onIconInteracted) onIconInteracted(path.slice(1), iconIndex);
  };

  const onAddClicked = () => {
    onAdd(selected?.slice(1) || []);
  };

  const onRemoveClicked = () => {
    onRemove(selected?.slice(1) || []);
    setSelected(undefined);
  };

  const onDuplicateClicked = () => {
    onDuplicate(selected?.slice(1) || []);
  };

  const onRenameClicked = () => {
    onRename(selected?.slice(1) || []);
  };

  return (
    <SidebarContainer>
      {" "}
      <TreeArea>
        <SortableTree
          items={tree}
          selected={selected}
          onSelected={onTreeNodeSelect}
          onIconSelected={onTreeNodeIconSelect}
        />
      </TreeArea>
      <PropertiesSectionDiv>{children}</PropertiesSectionDiv>
      <ButtonsDiv>
        <Button variant="contained" size="small" onClick={onAddClicked}>
          Add
        </Button>
        <Button
          variant="contained"
          size="small"
          disabled={selected === undefined || selected.length <= 1}
          onClick={onRemoveClicked}
        >
          Remove
        </Button>
        <Button
          variant="contained"
          size="small"
          disabled={selected === undefined || selected.length <= 1}
          onClick={onDuplicateClicked}
        >
          Duplicate
        </Button>
        <Button
          variant="contained"
          size="small"
          disabled={selected === undefined || selected.length <= 1}
          onClick={onRenameClicked}
        >
          Rename
        </Button>
      </ButtonsDiv>
    </SidebarContainer>
  );
}
