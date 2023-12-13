import { FC, useCallback  } from "react";
import styled from "@emotion/styled";
import { Button } from "@formant/ui-sdk";
import "./App.css";

interface IButtonRowProps {
  streamName: string;
  sendBtnPressFn: (streamName: string, state: boolean) => void;
}

export const ButtonRow: FC<IButtonRowProps> = ({
  sendBtnPressFn,
  streamName,
}) => {
  const issueBtnPress = useCallback(async () => {
    try {
      sendBtnPressFn(streamName, true);
    } catch (error) {
      throw error;
    }
  }, [sendBtnPressFn, streamName]);

  return (
    <Row>
      <Description>
        <Name>{streamName}</Name>
      </Description>
      <ButtonContainer>
        <Button
          onClick={issueBtnPress}
          color="primary"
          variant="contained"
          sx={{ minWidth: 155 }}
        >
          Button Press
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

const ButtonContainer = styled.td`
  width: auto;
  padding: 15px;
`;

const Name = styled.span`
  color: white;
`;
