import ModuleConfig from "../types/ModuleConfig";

const moduleConfig: ModuleConfig = {
  sections: [
    {
      title: "navigation",
      contents: [
        { topic: "/formant/ingest", minHz: 0 },
        { topic: "/rosout", minHz: 0 },
      ],
    },
    {
      title: "info",
      contents: [
        { topic: "/formant/command", minHz: 10 },
        { topic: "/tf", minHz: 10 },
      ],
    },
  ],
};
export default moduleConfig;
