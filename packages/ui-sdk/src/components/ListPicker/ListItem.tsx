import { Switch, Typography } from "../../main";
import React, { FC } from "react";
import styled from "@emotion/styled";

interface IListItemProps {
  name: string;
  enabled: boolean;
  onChange: (
    event: React.ChangeEvent<HTMLInputElement>,
    checked: boolean
  ) => void;
}

export const ListItem: FC<IListItemProps> = ({ name, enabled, onChange }) => {
  return (
    <Container>
      <Typography variant="body2">{name}</Typography>{" "}
      <Switch
        size="small"
        onChange={onChange}
        checked={enabled}
        value={enabled}
      />
    </Container>
  );
};

const Container = styled.div`
  display: flex;
  justify-content: space-between;
  border-bottom: 0.03938rem solid #282f45;
  height: 3.625rem;
  align-items: center;
`;
