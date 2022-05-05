import * as React from "react";
import { Icon, Tooltip, Typography } from "@formant/ui-sdk";
import styled from "styled-components";
import { TreeElement, TreePath } from "../model/ITreeElement";

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

export function SortableTree(props: ISortableTreeProps) {
  const onIconClicked = (path: TreePath, iconIndex: number) => {
    if (props.onIconSelected) {
      props.onIconSelected(path, iconIndex);
    }
  };

  const select = (path: TreePath) => {
    if (props.onSelected) {
      props.onSelected(path);
    }
  };

  const onItemClicked = (p: TreePath) => {
    select(p);
  };

  const onItemIconClicked = (currentPath: TreePath, iconIndex: number) => {
    onIconClicked(currentPath, iconIndex);
  };

  const renderTree = (elements: TreeElement[], pathSoFar: TreePath = []) => {
    const { selected } = props;
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
                onClick={onItemClicked.bind(undefined, currentPath)}
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
                        onClick={onItemIconClicked.bind(
                          undefined,
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
          <div>{e.children && renderTree(e.children, currentPath)}</div>
        </React.Fragment>
      );
    });
  };

  const { items } = props;
  return <div>{renderTree(items, [])}</div>;
}
