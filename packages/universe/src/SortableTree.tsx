import * as React from "react";
import { Icon, Tooltip, Typography } from "@formant/ui-sdk";
import { Component } from "react";
import styled from "styled-components";
import { TreeElement, TreePath } from "./ITreeElement";

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
  margin-bottom: 0rem;
  border-bottom: #3b4668 solid 1px;
`;

const IconsDiv = styled.div`
  float: right;
`;

export class SortableTree extends Component<ISortableTreeProps> {
  private onItemClicked = (p: TreePath) => {
    this.select(p);
  };

  private onItemIconClicked = (currentPath: TreePath, iconIndex: number) => {
    this.onIconClicked(currentPath, iconIndex);
  };

  onIconClicked(path: TreePath, iconIndex: number) {
    if (this.props.onIconSelected) {
      this.props.onIconSelected(path, iconIndex);
    }
  }

  select(path: TreePath) {
    if (this.props.onSelected) {
      this.props.onSelected(path);
    }
  }

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
        <React.Fragment key={`tree_item${currentPath.join("-")}`}>
          <TreeItemDiv>
            <span
              style={{
                marginLeft: `${(currentPath.length - 1) * 1}rem`,
                color: isSelected ? "#18d2ff" : "#BAC4E2",
              }}
            >
              <TitleSpan
                onClick={this.onItemClicked.bind(this, currentPath)}
                data-tooltip={e.title}
              >
                <Typography variant="body1">
                  {currentPath.length > 1 && <span>— </span>}

                  {e.title}
                  {e.textColor && (
                    <>
                      {" "}
                      <Icon
                        name="device"
                        sx={{
                          color: e.textColor,
                          width: "1rem",
                          height: "1rem",
                        }}
                      />
                    </>
                  )}
                </Typography>
              </TitleSpan>
            </span>
            <IconsDiv>
              {e.icons &&
                e.icons.map((icon, iconIndex) => (
                  <IconDiv
                    key={`tree_item_icon${currentPath.join("-")}-${currentPath
                      .map((_) => _.toString())
                      .join("-")}+${icon.icon}`}
                  >
                    <Tooltip title={icon.description}>
                      <div
                        tabIndex={iconIndex}
                        role="button"
                        onClick={this.onItemIconClicked.bind(
                          this,
                          currentPath,
                          iconIndex
                        )}
                      >
                        <Icon
                          name={icon.icon}
                          sx={{
                            color: icon.color,
                            width: "1rem",
                            height: "1rem",
                          }}
                        />
                      </div>
                    </Tooltip>
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
