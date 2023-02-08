import { Icon } from "../../main";
import React from "react";
import styled from "@emotion/styled";

export const SearchBar = (props: any) => {
  const { onChange, onClick, value } = props;

  return (
    <Container>
      <Icon name="search" />
      <Input value={value} onChange={onChange} {...props} />
      <Button onClick={onClick}>
        {value !== null && value !== "" && <Icon name="close" />}
      </Button>
    </Container>
  );
};

const Container = styled.div`
  box-sizing: border-box;
  position: relative;
  height: 38.18px;
  background-color: #3d4665;
  display: flex;
  transition: all ease-in 0.2s;
  border-radius: 25px;
  align-items: center;
  padding-left: 10px;
  padding-right: 10px;
  width: 100%;
  border: 1px solid transparent;
  &:focus-within {
    background-color: black;
    border: 1px solid #18d2ff;
  }
  svg {
    fill: #bac4e2;
  }
`;

const Input = styled.input`
background-color: transparent;
border: none;
outline: none;
color: #bac4e2;
height: 100%;
width: 100%;
font-family: "inter";
letter-spacing: .2px;
}
`;

const Button = styled.button`
  height: 100%;
  width: 24px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: transparent;
  outline: none;
  border: none;
`;
