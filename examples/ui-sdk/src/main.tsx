import * as React from "react";
import { createRoot } from "react-dom/client";
import {
  FormantProvider,
  Box,
  Button,
  Container,
  Grid,
  Typography,
  Select,
  TextField,
  Tooltip,
  HelpIcon,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogContentText,
  DialogActions,
  Switch,
  Stack,
  Link,
} from "@formant/ui-sdk";

function App() {
  const [open, setOpen] = React.useState(false);

  const handleClickOpen = () => {
    setOpen(true);
  };

  const handleClose = () => {
    setOpen(false);
  };

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
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Select
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <Select
                label="Age"
                items={[
                  { label: "0-10", value: "0-10" },
                  { label: "11-20", value: "11-20" },
                  { label: "21-30", value: "21-30" },
                ]}
              />
            </Grid>
          </Box>
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Text Input
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <TextField
                id="outlined-basic"
                label="Outlined"
                variant="filled"
              />
            </Grid>
          </Box>
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Tooltip
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <Tooltip title="Delete">
                <Button>Hover over me</Button>
              </Tooltip>
              <HelpIcon
                description={
                  <>
                    You can put a <Link href="https://formant.io">link</Link> in
                    here too.
                  </>
                }
              />
            </Grid>
          </Box>
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Tooltip
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <Button variant="outlined" onClick={handleClickOpen}>
                Open form dialog
              </Button>
              <Dialog open={open} onClose={handleClose}>
                <DialogTitle>Subscribe</DialogTitle>
                <DialogContent>
                  <Stack spacing={2}>
                    <DialogContentText>
                      To subscribe to this website, please enter your email
                      address here. We will send updates occasionally.
                    </DialogContentText>
                    <TextField
                      variant="filled"
                      autoFocus
                      id="name"
                      label="Email Address"
                      type="email"
                      fullWidth
                    />
                  </Stack>
                </DialogContent>
                <DialogActions>
                  <Stack direction="row" spacing={2}>
                    <Button
                      size="large"
                      variant="contained"
                      onClick={handleClose}
                    >
                      Cancel
                    </Button>
                    <Button
                      size="large"
                      variant="contained"
                      color="secondary"
                      onClick={handleClose}
                    >
                      Subscribe
                    </Button>
                  </Stack>
                </DialogActions>
              </Dialog>
            </Grid>
          </Box>
          <Typography variant="h2" sx={{ textDecoration: "underline" }}>
            Switch
          </Typography>
          <Box sx={{ p: 2 }}>
            <Grid container gap={2}>
              <Switch />
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
