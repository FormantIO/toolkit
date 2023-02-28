import { DiagnosticStatusMessage } from "./types/types";

export const dummyData: DiagnosticStatusMessage = {
  header: {
    frame_id: "",
    seq: 875,
    stamp: { secs: 1659048794, nsecs: 999167919 },
  },
  status: [
    {
      hardware_id: "ABC123",
      level: 0,
      message: "Normal operation",
      name: "To begin exploring your data, you can create a JSON stream.",
      values: [
        {
          key: "Angle",
          value: "45 degrees",
        },
        {
          key: "Speed",
          value: "10 RPM",
        },
      ],
    },
  ],
};
