import React from "react";
import styled from "styled-components";
import { TreePath } from "./ITreeElement";
import { UniverseSidebar } from "./sidebar";
import { UniverseViewer } from "./viewer";

const UniverseContainer = styled.div`
  width: 100%;
  height: 100vh;
`;

interface IUniverseProps {
  sidebar?: () => React.ReactNode;
}

export function Universe({ sidebar }: IUniverseProps) {
  return (
    <UniverseContainer>
      {sidebar ? (
        sidebar()
      ) : (
        <UniverseSidebar
          tree={[
            {
              title: "hello",
              textColor: "red",
              icons: [{ icon: "eye", description: "help", color: "yellow" }],
              children: [],
            },
            {
              title: "rest",
              textColor: "red",
              icons: [{ icon: "eye", description: "help", color: "yellow" }],
              children: [],
            },
          ]}
          onAdd={function (currentPath: TreePath): void {
            throw new Error("Function not implemented.");
          }}
          onRemove={function (currentPath: TreePath): void {
            throw new Error("Function not implemented.");
          }}
          onDuplicate={function (currentPath: TreePath): void {
            throw new Error("Function not implemented.");
          }}
          onRename={function (currentPath: TreePath): void {
            throw new Error("Function not implemented.");
          }}
          onItemSelected={function (currentPath?: any): void {
            throw new Error("Function not implemented.");
          }}
        />
      )}
      <UniverseViewer />
    </UniverseContainer>
  );
}
