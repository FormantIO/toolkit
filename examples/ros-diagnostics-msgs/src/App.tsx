import "./App.css";
import { Table } from "./components/table";
import useModuleDataListener from "./hooks/useDataModuleListener";
import { useFetchDiagnostics } from "./hooks/useFetchDiagnostics";
import { LoadingIndicator, useFormant } from "@formant/ui-sdk";
import { dummyData } from "./util";
import { IConfiguration } from "./types/types";
import { useEffect, useMemo } from "react";

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const diagnostics = useModuleDataListener();
  const messages = useFetchDiagnostics(diagnostics);

  const handleConfiguration =
    config.stream?.length > 1 ? messages : [dummyData];

  return (
    <div className="App">
      <Table messages={handleConfiguration} config={config} />
    </div>
  );
}

export default App;
