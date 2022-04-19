import * as React from "react";
import { Icon } from "@formant/ui-sdk";
import { Component } from "react";
import { TreeElement, TreePath } from "./ITreeElement";
import styled from "styled-components";

interface ISortableTreeProps {
  items: TreeElement[];
  selected?: TreePath;
  onSelected?: (item: TreePath) => void;
  onIconSelected?: (item: TreePath, iconIndex: number) => void;
}

const IconDiv = styled.div`
  > svg {
    margin-bottom: -0.3rem;
    margin-left: 0.3rem;
  }
  display: inline-block;
`;

const TitleSpan = styled.span`
  max-width: 20rem;
  text-overflow: ellipsis;
  display: inline-block;
  white-space: nowrap;
  overflow: hidden;
`;

const TreeItemDiv = styled.div`
  cursor: pointer;
  margin-top: 0.5rem;
  margin-bottom: 0.5rem;
  border-bottom: solid 1px;
  padding-bottom: 0.5rem;
`;

const IconsDiv = styled.div`
  float: right;
`;

export class SortableTree extends Component<ISortableTreeProps> {
  select(path: TreePath) {
    if (this.props.onSelected) {
      this.props.onSelected(path);
    }
  }

  onIconClicked(path: TreePath, iconIndex: number) {
    if (this.props.onIconSelected) {
      this.props.onIconSelected(path, iconIndex);
    }
  }

  private onItemClicked = (p: TreePath) => {
    this.select(p);
  };

  private onItemIconClicked = (currentPath: TreePath, iconIndex: number) => {
    this.onIconClicked(currentPath, iconIndex);
  };

  renderTree(elements: TreeElement[], pathSoFar: TreePath = []) {
    const { selected } = this.props;
    return elements.map((e: TreeElement, elementIndex: number) => {
      const currentPath = [...pathSoFar, elementIndex];
      const isSelected =
        selected &&
        selected.length === currentPath.length &&
        selected.every(
          (val, selectedIndex) => val === currentPath[selectedIndex]
        );
      return (
        <React.Fragment key={"tree_item" + currentPath.join("-")}>
          <TreeItemDiv>
            <span
              style={{
                marginLeft: `${(currentPath.length - 1) * 2}rem`,
                color: isSelected ? "blue" : "inherit",
              }}
            >
              <TitleSpan
                onClick={this.onItemClicked.bind(this, currentPath)}
                data-tooltip={e.title}
              >
                {currentPath.length > 1 && <span>- </span>}

                {e.title}
                {e.textColor && (
                  <span style={{ color: e.textColor }}> &#9675;</span>
                )}
              </TitleSpan>
            </span>
            <IconsDiv>
              {e.icons &&
                e.icons.map((icon, iconIndex) => (
                  <IconDiv
                    key={
                      "tree_item_icon" + currentPath.join("-") + "-" + iconIndex
                    }
                  >
                    <Icon
                      name={icon.icon}
                      size="18"
                      data-tooltip={icon.description}
                      onClick={this.onItemIconClicked.bind(
                        this,
                        currentPath,
                        iconIndex
                      )}
                      color={icon.color}
                    />
                  </IconDiv>
                ))}
            </IconsDiv>
          </TreeItemDiv>
          <div>{e.children && this.renderTree(e.children, currentPath)}</div>
        </React.Fragment>
      );
    });
  }

  public render() {
    const { items } = this.props;
    return <div>{this.renderTree(items, [])}</div>;
  }
}
