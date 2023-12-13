import { FC } from "react";
import styled from "@emotion/styled";
import "./App.css";

interface IButtonRowProps {
  streamName: string;
}

export const ButtonRow: FC<IButtonRowProps> = ({
  streamName,
}) => {
  return (
    <Row>
      <Description>
        <Name>{streamName}</Name>
      </Description>
    </Row>
  );
};

const Row = styled.tr`
  width: 100%;
  border-bottom: 0.039375rem solid #1c1e2d;
`;

const Description = styled.td`
  text-align: left;
  width: auto;
  padding: 5px;
`;

const Name = styled.span`
  color: white;
`;
