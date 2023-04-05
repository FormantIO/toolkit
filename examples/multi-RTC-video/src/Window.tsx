import styled from "@emotion/styled";
import { useCallback, useEffect, useState, ReactNode, FC } from "react";

interface IWindowProps {
  children: ReactNode;
}

export const Window: FC<IWindowProps> = ({ children }) => {
  const [height, setHeight] = useState("auto");
  const [width, setWidth] = useState("50%");
  const [position, setPosition] = useState("relative");
  const [left, setLeft] = useState();
  const [top, setTop] = useState();

  const handleFullScreen = useCallback(() => {
    setHeight("100vh");
    setWidth("100vw");
    setPosition("absolute");
    setTop(0);
    setLeft(0);
  }, []);

  useEffect(() => {
    console.log("doneeee")
    window.addEventListener("dblclick", handleFullScreen);

    return () => {
      window.removeEventListener("dblclick", handleFullScreen);
    };
  }, []);

  return (
    <Cell style={{ height, width, position: position as any, left, top }}>
      {children}
    </Cell>
  );
};

const Cell = styled.div`
  background: black;
  width: 50%;
`;
