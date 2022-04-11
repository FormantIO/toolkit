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
  TextField,
  Toolbar,
  Typography,
} from "@mui/material";
import { createRoot } from "react-dom/client";
import { defaultTheme } from "../../../packages/formant-theme-material-ui/src/main";
import MenuIcon from "@mui/icons-material/Menu";
import formantLogo from "./formant.svg";
import DeleteIcon from "@mui/icons-material/Delete";

const theme = createTheme(defaultTheme);

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
        <Container maxWidth="lg">
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Typography
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid
              container
              justifyContent="center"
              alignItems="center"
              rowGap={2}
            >
              <Grid item xs={4}>
                <Typography variant="h4">H1</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="h1">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">H2</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="h2">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">H3</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="h3">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">H4</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="h4">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">H5</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="h5">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">H6</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="h6">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">body1</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="body1">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">body2</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="body2">0123456789</Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="h4">caption</Typography>
              </Grid>
              <Grid item xs={8}>
                <Typography variant="caption">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
            </Grid>
          </Box>
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Buttons
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <Button variant="text" size="small">
                Text
              </Button>
              <Button variant="contained" size="small">
                Contained
              </Button>
              <Button variant="contained" size="small" color="secondary">
                Contained
              </Button>
              <Button
                variant="contained"
                size="small"
                color="error"
                startIcon={<DeleteIcon />}
              >
                Contained
              </Button>
              <Button variant="outlined" size="small">
                Outlined
              </Button>
            </Grid>
            <br />
            <Grid container gap={2}>
              <Button variant="text">Text</Button>
              <Button variant="contained">Contained</Button>
              <Button variant="contained" color="secondary">
                Contained
              </Button>
              <Button
                variant="contained"
                color="error"
                startIcon={<DeleteIcon />}
              >
                Contained
              </Button>
              <Button variant="outlined">Outlined</Button>
            </Grid>
            <br />
            <Grid container gap={2}>
              <Button variant="text" size="large">
                Text
              </Button>
              <Button variant="contained" size="large">
                Contained
              </Button>
              <Button variant="contained" size="large" color="secondary">
                Contained
              </Button>
              <Button
                variant="contained"
                size="large"
                color="error"
                startIcon={<DeleteIcon />}
              >
                Contained
              </Button>
              <Button variant="outlined" size="large">
                Outlined
              </Button>
            </Grid>
          </Box>
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Text Fields
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <TextField
                required
                id="outlined-required"
                label="Required"
                defaultValue="Hello World"
              />
              <TextField
                disabled
                id="outlined-disabled"
                label="Disabled"
                defaultValue="Hello World"
              />
              <TextField
                id="outlined-password-input"
                label="Password"
                type="password"
                autoComplete="current-password"
              />
              <TextField
                id="outlined-read-only-input"
                label="Read Only"
                defaultValue="Hello World"
                InputProps={{
                  readOnly: true,
                }}
              />
              <TextField
                id="outlined-number"
                label="Number"
                type="number"
                InputLabelProps={{
                  shrink: true,
                }}
              />
              <TextField
                id="outlined-search"
                label="Search field"
                type="search"
              />
              <TextField
                id="outlined-helperText"
                label="Helper text"
                defaultValue="Default Value"
                helperText="Some important text"
              />
            </Grid>
            <Grid container gap={2}>
              <TextField
                required
                id="filled-required"
                label="Required"
                defaultValue="Hello World"
                variant="filled"
              />
              <TextField
                disabled
                id="filled-disabled"
                label="Disabled"
                defaultValue="Hello World"
                variant="filled"
              />
              <TextField
                id="filled-password-input"
                label="Password"
                type="password"
                autoComplete="current-password"
                variant="filled"
              />
              <TextField
                id="filled-read-only-input"
                label="Read Only"
                defaultValue="Hello World"
                InputProps={{
                  readOnly: true,
                }}
                variant="filled"
              />
              <TextField
                id="filled-number"
                label="Number"
                type="number"
                InputLabelProps={{
                  shrink: true,
                }}
                variant="filled"
              />
              <TextField
                id="filled-search"
                label="Search field"
                type="search"
                variant="filled"
              />
              <TextField
                id="filled-helperText"
                label="Helper text"
                defaultValue="Default Value"
                helperText="Some important text"
                variant="filled"
              />
            </Grid>
            <Grid container gap={2}>
              <TextField
                required
                id="standard-required"
                label="Required"
                defaultValue="Hello World"
                variant="standard"
              />
              <TextField
                disabled
                id="standard-disabled"
                label="Disabled"
                defaultValue="Hello World"
                variant="standard"
              />
              <TextField
                id="standard-password-input"
                label="Password"
                type="password"
                autoComplete="current-password"
                variant="standard"
              />
              <TextField
                id="standard-read-only-input"
                label="Read Only"
                defaultValue="Hello World"
                InputProps={{
                  readOnly: true,
                }}
                variant="standard"
              />
              <TextField
                id="standard-number"
                label="Number"
                type="number"
                InputLabelProps={{
                  shrink: true,
                }}
                variant="standard"
              />
              <TextField
                id="standard-search"
                label="Search field"
                type="search"
                variant="standard"
              />
              <TextField
                id="standard-helperText"
                label="Helper text"
                defaultValue="Default Value"
                helperText="Some important text"
                variant="standard"
              />
            </Grid>
          </Box>
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
