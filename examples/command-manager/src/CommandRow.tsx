import { FC, useCallback, useEffect, useMemo, useState } from "react";
import styled from "@emotion/styled";
import { TextField, Button, Typography, Select } from "@formant/ui-sdk";
import { Authentication, Device, Fleet } from "@formant/data-sdk";
import { useDropDownItems } from "./hooks/useDropDownItem";
import { IStream } from "./App";
import "./App.css";
interface ICommandRowProps {
  name: string;
  enabledParameters: boolean;
  description: string;
  useStreamValue?: boolean;
  streamName?: string;
  device?: Device;
  streams?: IStream[];
}

const setStreamAsActive = async (stream: any) => {
  if (await Authentication.waitTilAuthenticated()) {
    await Fleet.patchStream({ ...stream, active: true });
  }
};

export const CommandRow: FC<ICommandRowProps> = ({
  name,
  description,
  streamName,
  device,
  streams,
  enabledParameters,
  useStreamValue,
}) => {
  const [param, setParam] = useState("");
  const [disabled, setDisabled] = useState(false);
  const dropdownItem = useDropDownItems(device, streamName);
  const [responsiveDescription, setResponisveDescription] = useState("");

  const handleResponsiveDescription = useCallback(() => {
    if (description?.length === 0) return;
    if (window.innerWidth < 635) {
      setResponisveDescription(`${description?.slice(0, 7)}... `);
      return;
    }
    if (window.innerWidth < 860) {
      setResponisveDescription(`${description?.slice(0, 15)}... `);
      return;
    }
    setResponisveDescription(description);
  }, [description]);

  useEffect(() => {
    if (!description) return;
    handleResponsiveDescription();
    window.addEventListener("resize", handleResponsiveDescription);
  }, [description]);

  useEffect(() => {
    if (!streams || !streamName) return;
    setStreamAsActive(streams.filter((_) => _.streamName === streamName)[0]);
  }, [streamName]);

  const issueCommand = useCallback(async () => {
    try {
      setDisabled(true);
      if (!device) return;

      device.sendCommand(name, param ?? "");
      setTimeout(() => {
        setDisabled(false);
      }, 3000);
    } catch (error) {
      throw error;
    }
  }, [device, param]);

  return (
    <Row>
      <Description>
        <Name>{name}</Name>
        <Typography variant="body1">{responsiveDescription}</Typography>
      </Description>
      <InputContainer>
        {dropdownItem.length > 0 && (
          <Select
            label="Parameters"
            items={dropdownItem}
            onChange={(value) => setParam(value as string)}
          />
        )}
      </InputContainer>
      {enabledParameters && !useStreamValue && (
        <InputContainer>
          <TextField
            variant="filled"
            label="Parameters"
            value={param}
            onChange={(e) => setParam(e.target.value)}
          />
        </InputContainer>
      )}
      <ButtonContainer>
        <Button
          disabled={disabled}
          onClick={issueCommand}
          color="primary"
          variant="contained"
          sx={{ minWidth: 155 }}
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
  text-align: left;
  width: auto;
  padding: 5px;
`;

const InputContainer = styled.td`
  width: auto;
  padding: 5px;
  display: flex;
  align-items: center;
`;

const ButtonContainer = styled.td`
  width: auto;
  padding: 15px;
`;

const Name = styled.span`
  color: white;
`;
