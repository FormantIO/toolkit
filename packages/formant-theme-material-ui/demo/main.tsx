import * as React from "react";
import Button from "@mui/material/Button";
import ThemeProvider from "@mui/material/styles/ThemeProvider";
import CssBaseline from "@mui/material/CssBaseline";
import {
  AppBar,
  Box,
  Card,
  Container,
  createTheme,
  Grid,
  IconButton,
  Toolbar,
  Typography,
} from "@mui/material";
import { createRoot } from "react-dom/client";
import { lightTheme } from "../src/main";
import MenuIcon from "@mui/icons-material/Menu";
import formantLogo from "./formant.svg";

const theme = createTheme(lightTheme);

function App() {
  return (
    <div>
      <AppBar position="static">
        <Toolbar variant="regular">
          <IconButton edge="start" aria-label="menu" sx={{ mr: 2 }}>
            <MenuIcon />
          </IconButton>
          <Box
            component="img"
            sx={{
              height: 30,
            }}
            alt="The house from the offer."
            src={formantLogo}
          />
        </Toolbar>
      </AppBar>
      <Box sx={{ p: 4 }}>
        <Container maxWidth="sm">
          <Button variant="text">Text</Button>
          <Button variant="contained">Contained</Button>
          <Button variant="contained" color="secondary">
            Contained
          </Button>
          <Button variant="contained" color="error">
            Contained
          </Button>
          <Button variant="outlined">Outlined</Button>
          <Grid sx={{ pt: 4, pb: 4 }}>
            <Card sx={{ borderRadius: 0.5 }}>
              <Box sx={{ p: 2 }}>
                <Typography variant="body1">
                  Something important goes here
                </Typography>
              </Box>
            </Card>
          </Grid>
        </Container>
      </Box>
    </div>
  );
}

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <ThemeProvider theme={theme}>
      <CssBaseline>
        <App />
      </CssBaseline>
    </ThemeProvider>
  );
}
