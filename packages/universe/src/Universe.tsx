import React from "react";
import styled from "styled-components";
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
      {sidebar ? sidebar() : <UniverseSidebar />}
      <UniverseViewer />
    </UniverseContainer>
  );
}
