import * as React from "react";
import { createRoot } from "react-dom/client";
import {
  FormantProvider,
  Box,
  Button,
  Container,
  Grid,
  Typography,
} from "../src/main";

function App() {
  return (
    <div>
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
              <Grid item xs={12} md={4}>
                <Typography variant="h4">H1</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="h1">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">H2</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="h2">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">H3</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="h3">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">H4</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="h4">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">H5</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="h5">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">H6</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="h6">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">body1</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="body1">
                  The quick brown fox jumps over the lazy dog.
                </Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">body2</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
                <Typography variant="body2">0123456789</Typography>
              </Grid>
              <Grid item xs={12} md={4}>
                <Typography variant="h4">caption</Typography>
              </Grid>
              <Grid item xs={12} md={8}>
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
              <Button variant="contained" size="small" color="error">
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
              <Button variant="contained" color="error">
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
              <Button variant="contained" size="large" color="error">
                Contained
              </Button>
              <Button variant="outlined" size="large">
                Outlined
              </Button>
            </Grid>
          </Box>
        </Container>
      </Box>
    </div>
  );
}

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider>
      <App />
    </FormantProvider>
  );
}
