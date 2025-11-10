import {
  useDevice,
  Box,
  Button,
  Typography,
  Select,
  TextField,
  Snackbar,
} from "@formant/ui-sdk";
import {
  useCallback,
  useState,
  useMemo,
  useEffect,
  FormEventHandler,
} from "react";
import { getDevice, sendCommand } from "./utils";

interface Topic {
  name: string;
  type: string;
}

export const Form = () => {
  const device = useDevice();
  const [topics, setTopics] = useState<Topic[]>([]);
  const [topic, setTopic] = useState("");
  const [param, setParam] = useState<null | string>("");
  const [showSnackbar, setShowSnackbar] = useState(false);

  useEffect(() => {
    if (!device) return;
    getDevice(device.id).then((_topics) => setTopics(_topics));
  }, [device]);

  const handleIssueCommand = useCallback<FormEventHandler<HTMLFormElement>>(
    async (ev) => {
      ev.preventDefault();
      if (!device || !param) return;
      sendCommand(device as any, "publish.string", topic, param);
      setShowSnackbar(true);
      setParam("");
      setTopic("");
    },
    [topic, param]
  );

  const handleOnChange = useCallback((val: string) => {
    setTopic(val);
    setParam("");
  }, []);

  const dropdownItems = useMemo(
    () =>
      topics.map((_) => ({
        label: _.name,
        value: _.name,
      })),
    [topics]
  );
  return (
    <form onSubmit={handleIssueCommand}>
      <Box position="relative" display="flex" flexDirection="column">
        <Typography variant="h3">Publish string to ROS topic</Typography>
        <Select
          sx={{ marginBottom: 2, marginTop: 2 }}
          items={dropdownItems}
          value={topic}
          onChange={handleOnChange}
          label={"ROS topics"}
        />
        <TextField
          label="String"
          variant="filled"
          value={param}
          onChange={(ev) => setParam(ev.target.value)}
        />

        <Button
          type="submit"
          sx={{ position: "absolute", bottom: -60, right: 0 }}
          size="large"
          variant="contained"
          color="secondary"
          disabled={param === null || param.length === 0 || topic.length === 0}
        >
          Publish
        </Button>
        <Snackbar
          message="String published"
          open={showSnackbar}
          onClose={() => setShowSnackbar(false)}
          autoHideDuration={3000}
        />
      </Box>
    </form>
  );
};
