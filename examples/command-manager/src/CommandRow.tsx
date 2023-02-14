import { FC, useState } from "react";
import styled from "@emotion/styled";
import { TextField, Button, Typography } from "@formant/ui-sdk";
interface ICommandRowProps {
  name: string;
  description?: string;
  streamName?: string;
}

export const CommandRow: FC<ICommandRowProps> = ({
  name,
  description,
  streamName,
}) => {
  const [param, setParam] = useState("");

  return (
    <Row>
      <Description>
        <Name>{name}</Name>
        <Typography variant="body1">{description}</Typography>
      </Description>
      <InputContainer>
        <TextField />
      </InputContainer>
      <ButtonContainer>
        <Button
          color="primary"
          variant="contained"
          sx={{
            marginLeft: 1,
            marginRight: 1,
          }}
        >
          Issue Command
        </Button>
      </ButtonContainer>
    </Row>
  );
};

const Row = styled.tr`
  width: 100%;
  border-bottom: 0.039375rem solid #1c1e2d;
`;

const Description = styled.td`
  width: auto;
`;

const InputContainer = styled.td`
  width: auto;
`;

const ButtonContainer = styled.td`
  width: auto;
`;

const Name = styled.span`
  color: white;
`;
