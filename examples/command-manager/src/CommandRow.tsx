import { FC, useState } from "react";
import styled from "@emotion/styled";
import { TextField, Button, Typography, Select } from "@formant/ui-sdk";
import { Authentication, Device } from "@formant/data-sdk";
import { useStreams } from "./hooks/useStreams";
interface ICommandRowProps {
  name: string;
  description?: string;
  streamName?: string;
  device?: Device;
}

const setStreamAsActive = async (stream: any) => {
  if (await Authentication.waitTilAuthenticated()) {
    const response = await fetch(
      `https://api.formant.io/v1/admin/streams/${stream.id}`,
      {
        method: "PATCH",
        body: JSON.stringify({ ...stream, active: true }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
  }
};

export const CommandRow: FC<ICommandRowProps> = ({
  name,
  description,
  streamName,
  device,
}) => {
  const [param, setParam] = useState("");

  return (
    <Row>
      <Description>
        <Name>{name}</Name>
        <Typography variant="body1">{description}</Typography>
      </Description>
      {!streamName ? (
        <Select label="Parameters" fullWidth />
      ) : (
        <InputContainer>
          <TextField label="Parameters" />
        </InputContainer>
      )}
      <ButtonContainer>
        <Button color="primary" variant="contained" sx={{ minWidth: 155 }}>
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
  text-align: left;
  width: auto;
  padding: 5px;
  min-width: 375px;
`;

const InputContainer = styled.td`
  width: auto;
  padding: 5px;
  min-width: 215px;
`;

const ButtonContainer = styled.td`
  width: auto;
  padding: 5px;
`;

const Name = styled.span`
  color: white;
`;
