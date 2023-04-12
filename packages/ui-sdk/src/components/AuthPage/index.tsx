import styled from "@emotion/styled";
import React, { FC, useCallback, useState } from "react";
import { TextField, SxProps, Typography } from "@mui/material";
import Logo from "../../images/loginlogo.svg";
import googleIcon from "../../images/google.svg";
import { Authentication, Fleet } from "@formant/data-sdk";

interface IAuthPageProps {
  textFieldType?: "filled" | "outlined" | "standard" | "no-styled";
  backgroundColor?: string;
  backgroundImage?: string;
  cardBackgroundColor?: string;
  inputSXProps?: SxProps;
  cardSXProps?: React.CSSProperties;
  loginButtonProps?: React.CSSProperties;
  logoProps?: React.CSSProperties;
  signInWithGoogleButtonProps?: React.CSSProperties;
  allowGoogleSignIn?: boolean;
}

export const AuthPage: FC<IAuthPageProps> = ({
  textFieldType = "filled",
  backgroundColor,
  cardBackgroundColor,
  inputSXProps,
  cardSXProps,
  logoProps,
  backgroundImage,
  loginButtonProps,
  signInWithGoogleButtonProps,
  allowGoogleSignIn,
}) => {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");

  const handleLogin = useCallback(
    async (e: React.FormEvent<HTMLFormElement>) => {
      e.preventDefault();
      await Authentication.login(email, password);
      const dev = await Fleet.getDevices();
      console.log(dev);
    },
    [email, password]
  );

  return (
    <Container
      style={{
        backgroundColor,
        backgroundImage: `url(${backgroundImage})`,
        backgroundSize: "cover",
        backgroundRepeat: "no-repeat",
      }}
    >
      <Card
        onSubmit={handleLogin}
        style={{ ...cardSXProps, backgroundColor: cardBackgroundColor }}
      >
        <img style={logoProps} src={Logo} />
        <Typography>Sign in</Typography>
        {textFieldType === "standard" && <label htmlFor="">Email</label>}
        <BasicInput />
        <TextField
          sx={{
            backgroundColor: "#ffd60a",
            border: "3px solid #001d3d",
            borderRadius: 8,
            "& div": {
              backgroundColor: "transparent",
              border: "none",
            },
            "&:focus-within div": {
              backgroundColor: "transparent",
              border: "none",
            },
            "&:focus-within label": {
              backgroundColor: "white",
              border: "none",
              color: "black",
            },
            ...inputSXProps,
          }}
          InputLabelProps={{
            sx: {
              color: "black",
              textTransform: "capitalize",
              backgroundColor: "white",
              marginLeft: 2,
              "& :focus ": {
                backgroundColor: "white",
                border: "none",
              },
            },
          }}
          inputProps={{
            sx: {
              color: "black",
              paddingLeft: "15px",
              fontSize: "20px",
              backgroundColor: "white",
              borderRadius: 8,
            },
          }}
          // sx={inputSXProps}
          variant={textFieldType}
          label="Email"
        ></TextField>
        {textFieldType === "standard" && <label>Password</label>}
        <TextField
          sx={{
            backgroundColor: "blue",
            border: "3px solid #001d3d",
            borderRadius: 8,
            "& > div": {
              backgroundColor: "transparent",
              border: "transparent",
              borderBottom: "none !important",
            },
            "& > input::after": {
              borderBottom: "none !important",
            },
            "& > div::before": {
              border: "none",
            },
            "& div:hover::before": {
              borderBottom: "none !important",
            },
            "&:focus-within div": {
              backgroundColor: "transparent",
              border: "none",
            },
            "&:focus-within  div::before": {
              border: "none !important",
            },
            "&:focus-within > div > input::after": {
              borderBottom: "1px red solid",
            },
            "&:focus-within label": {
              backgroundColor: "tranparent",
              border: "none",
              color: "red",
            },
          }}
          inputProps={{
            underline: {
              "&&&:before": {
                borderBottom: "none",
              },
              "&&:after": {
                borderBottom: "none",
              },
            },
          }}
          label={"text"}
          // sx={inputSXProps}
          type="text"
          variant={"filled"}
        ></TextField>
        <ForgotPasswordButtonContainer>
          <ForgotPasswordButton>Forgot password?</ForgotPasswordButton>
        </ForgotPasswordButtonContainer>
        <LoginButton type="submit" style={loginButtonProps}>
          Sign In
        </LoginButton>
        {allowGoogleSignIn && (
          <LoginWithGoogleButton style={signInWithGoogleButtonProps}>
            <GoogleIcon src={googleIcon} />
            Continue with Google
          </LoginWithGoogleButton>
        )}
      </Card>
    </Container>
  );
};

const BasicInput = styled.input`
  all: unset;
`;

const GoogleIcon = styled.img`
  margin-right: 10px;
`;

const LoginWithGoogleButton = styled.button`
  all: unset;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 10px 0px;
  :hover {
    cursor: pointer;
  }
`;

const ForgotPasswordButtonContainer = styled.div`
  display: flex;
  align-items: center;
  justify-content: flex-end;
  width: 100%;
`;

const ForgotPasswordButton = styled.button`
  all: unset;
  :hover {
    cursor: pointer;
  }
`;

const LoginButton = styled.button`
  all: unset;
  width: 100%;
  display: grid;
  place-items: center;
  position: absolute;
  bottom: 0;
  left: 0;
  :hover {
    cursor: pointer;
  }
`;

const Container = styled.div`
  height: 100vh;
  width: 100vw;
  display: grid;
  place-items: center;
`;

const Card = styled.form`
  display: flex;
  flex-direction: column;
  justify-content: center;
  position: relative;
  box-shadow: 0 20px 30px rgba(0, 66, 89, 0.25);
`;
