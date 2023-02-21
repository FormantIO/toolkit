import styled from "@emotion/styled";
import { SerializedStyles } from "@emotion/react";
import React, { FC } from "react";
import { primary, spacing } from "@style/common";
import classNames from "classnames";
import { css } from "@emotion/css";

const positions = ["top", "left", "bottom", "right"] as const;
type Position = typeof positions[number];

export interface ILabelProps {
  position: Position;
  active: boolean;
  children?: React.ReactNode;
}

export const Label: FC<ILabelProps> = ({ position, active, children }) => {
  return (
    <Container className={classNames(styles[position], active && on)}>
      {children}
    </Container>
  );
};

const top = css`
  left: calc(50% - 0.625rem);
  top: ${spacing.space10};
`;
const bottom = css`
  left: calc(50% - 0.625rem);
  bottom: ${spacing.space10};
`;
const left = css`
  top: calc(50% - 0.625rem);
  left: ${spacing.space10};
`;
const right = css`
  top: calc(50% - 0.625rem);
  right: ${spacing.space10};
`;

const on = css`
  opacity: 1;
  color: white;
`;
const styles: { [key in Position]: any } = {
  top,
  left,
  right,
  bottom,
};

const Container = styled.div`
  align-items: center;
  height: 1.25rem;
  width: 1.25rem;
  font-size: 0.8125rem;
  color: ${primary.foreground};
  position: absolute;
  opacity: 0.5;
`;
