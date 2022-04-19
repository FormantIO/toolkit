import React from "react";
import styled from "styled-components";

const SidebarContainer = styled.div`
  width: 300px;
  height: 300px;
  position: absolute;
  background: red;
  top: 200px;
`;

export function UniverseSidebar() {
  return <SidebarContainer></SidebarContainer>;
}
