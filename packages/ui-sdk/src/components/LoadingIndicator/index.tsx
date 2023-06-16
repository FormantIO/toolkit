import React from "react";
import loading from "../../images/loading.png";
import styled from "@emotion/styled";

export const LoadingIndicator = () => {
  return (
    <Container>
      <img src={loading} />
    </Container>
  );
};

const Container = styled.div`
  height: 24px;
  width: 24px;
  overflow: hidden;
  img {
    height: 100%;
  }
`;
